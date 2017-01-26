// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "MapGen.h"



// Sets default values
AMapGen::AMapGen()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AMapGen::BeginPlay()
{
	Super::BeginPlay();

	FString problem = FString("problem_B");
	readJson(problem);

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		for (int j = 0; j < allGroundPoints.Num();++j) {

			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allGroundPoints[j][0], FRotator::ZeroRotator, spawnParams);
			allPolygons.Add(newActor);
			newActor->init(allGroundPoints[j], FVector(0,0,0));

		}

		for (int j = 0; j < allWallPoints.Num();++j) {
			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allWallPoints[j][0], FRotator::ZeroRotator, spawnParams);
			newActor->init(allWallPoints[j], FVector(0,0,0));
			allWalls.Add(newActor);
		}
	}

	Node pathNode = dijkstras();
	TArray<FVector> Path = getPath(pathNode.path);
	for (int i = 0; i < Path.Num(); i++) {
		print(Path[i].ToString());
		//GEngine->AddOnScreenDebugMessage(-1, 500.f, FColor::Cyan, Path[i].ToString());
	}
	
}

// Called every frame
void AMapGen::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
}

void AMapGen::readJson(FString fileName)
{
	Nvertices = 0;

	FString FullPath = FPaths::GameDir() + "Data/" + fileName + ".json";
	FString JsonStr;
	FFileHelper::LoadFileToString(JsonStr, *FullPath);
	TSharedRef<TJsonReader<TCHAR>> JsonReader = FJsonStringReader::Create(JsonStr);
	TSharedPtr<FJsonObject> JsonObject = MakeShareable(new FJsonObject());
	bool serialized = FJsonSerializer::Deserialize(JsonReader, JsonObject);

	if (!serialized) {
		return;
	}

	//pos / velocity
	TArray <TSharedPtr < FJsonValue >> vals = JsonObject->GetArrayField("goal_pos");
	goal_pos = FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z);

	vals = JsonObject->GetArrayField("goal_vel");
	goal_vel = FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z);

	vals = JsonObject->GetArrayField("start_vel");
	start_vel = FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z);

	vals = JsonObject->GetArrayField("start_pos");
	start_pos = FVector(-(float)vals[0]->AsNumber()*scale, (float)vals[1]->AsNumber()*scale, default_Z);

	allPoints.Add(start_pos);

	int i = 0;
	while (true) {
		FString fieldName = FString("polygon") + FString::FromInt(i);
		if (!JsonObject->HasField(fieldName)) {
			break;
		}

		TArray < TSharedPtr < FJsonValue > > coords = JsonObject->GetArrayField(fieldName);
		TArray<FVector> groundPoints;

		for (int j = 0; j < coords.Num(); ++j) {

			TSharedPtr < FJsonValue > test = coords[j];
			TArray <TSharedPtr < FJsonValue >> test2 = test->AsArray();
			TSharedPtr < FJsonValue> testX = test2[0];
			float x = (float)testX->AsNumber();

			TSharedPtr < FJsonValue> testY = test2[1];
			float y = (float)testY->AsNumber();

			groundPoints.Add(FVector(-x*scale, y*scale, default_Z));
			allPoints.Add(FVector(-x*scale, y*scale, default_Z));
			//GEngine->AddOnScreenDebugMessage(-1, 50.f, FColor::Cyan, FVector(-x*scale, y*scale, 0).ToString());//FString::SanitizeFloat(Hit.Distance));
			
			Nvertices++;
		}

		allGroundPoints.Add(groundPoints);

		++i;
	}

	// wall points
	FString fieldName = FString("boundary_polygon");
	/*if (!JsonObject->HasField(fieldName)) {
	break;
	}*/
	TArray < TSharedPtr < FJsonValue > > coords = JsonObject->GetArrayField(fieldName);
	TArray<FVector> wallPoints;

	for (int j = 0; j < coords.Num(); ++j) {

		TSharedPtr < FJsonValue > test = coords[j];
		TArray <TSharedPtr < FJsonValue >> test2 = test->AsArray();
		TSharedPtr < FJsonValue> testX = test2[0];
		float x = (float)testX->AsNumber();

		TSharedPtr < FJsonValue> testY = test2[1];
		float y = (float)testY->AsNumber();

		wallPoints.Add(FVector(-x*scale, y*scale, default_Z));
		allPoints.Add(FVector(-x*scale, y*scale, default_Z));

		Nvertices++;
	}

	for (int j = 0; j < wallPoints.Num() - 1; j++) {
		TArray<FVector> temp;
		temp.Add(wallPoints[j]);
		temp.Add(wallPoints[j + 1]);
		allWallPoints.Add(temp);
	}

	TArray<FVector> temp;
	temp.Add(wallPoints[wallPoints.Num() - 1]);
	temp.Add(wallPoints[0]);
	allWallPoints.Add(temp);

	//floats
	L_car = JsonObject->GetNumberField("L_car");
	a_max = JsonObject->GetNumberField("a_max");
	k_friction = JsonObject->GetNumberField("k_friction");
	omega_max = JsonObject->GetNumberField("omega_max");
	phi_max = JsonObject->GetNumberField("phi_max");
	v_max = JsonObject->GetNumberField("v_max");

	allPoints.Add(goal_pos);
}

Node AMapGen::dijkstras() {

	//1) Initialize distances of all vertices as infinite.
	std::vector<float> dist(allPoints.Num());
	dist[0] = 0;
	for (int i = 1; i<allPoints.Num(); i++) {
		dist[i] = 100000000;
	}

	/*2) Create an empty priority_queue pq.Every item
	of pq is a pair(weight, vertex).Weight(or
	distance) is used used as first item  of pair
	as first item is by default used to compare
	two pairs*/
	std::priority_queue<Node> Q;

	Node node;
	
	node.dist = 0;
	node.path.push_back(PolyPoint(-1,0)); // -1,0 is start, -1,1 is goal
	Q.push(node);


	Node node2;
	while (!Q.empty()) {
		node = Q.top();
		Q.pop();

		// loop over polygons
		for (int i = 0; i < allGroundPoints.Num(); ++i) {
			// loop over points in polygon
			for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
				// if point not in current path
				if (std::find(node.path.begin(), node.path.end(), PolyPoint(i, j)) == node.path.end()) {
					// if same polygon
					if (node.path.back().polygon_index == i) {
						// set ignore polygon i
					}

					float distance = Trace(getPoint(node.path.back()), allGroundPoints[i][j],IGNORE POLYGON i FLAG);


					float exact_distance = FVector::Dist(getPoint(node.path.back()), allGroundPoints[i][j]);
					float dist_error = abs(distance - exact_distance) / exact_distance;

					if (distance == 0 || dist_error < 0.1) {
						node2.dist = exact_distance + node.dist;
						node2.path = node.path;
						node2.path.push_back(PolyPoint(i,j));

						if (i == allPoints.Num() - 1) {
							return node2;
						}

						Q.push(node2);
					}
				}
			}			
		}
	}
	return Node();
}

float AMapGen::Trace(FVector start, FVector end) {
	FCollisionQueryParams TraceParams(FName(TEXT("")), true, this);
	TraceParams.bTraceAsyncScene = true;
	TraceParams.bReturnPhysicalMaterial = false;
	TraceParams.bTraceComplex = true;

	FHitResult Hit(ForceInit);
	//GetWorld()->LineTraceSingleByObjectType(Hit, FVector(-300, 400, 10), FVector(-1500, 400, 10), FCollisionObjectQueryParams(), FCollisionQueryParams());
	GetWorld()->LineTraceSingleByObjectType(Hit, start, end, FCollisionObjectQueryParams(), FCollisionQueryParams());
	
	float dist = Hit.Distance;
	if (end.X == -500)
		GEngine->AddOnScreenDebugMessage(-1, 50.f, FColor::Red, start.ToString() + " " + end.ToString() + " " +FString::SanitizeFloat(dist));

	return dist;
}

TArray<FVector> AMapGen::getPath(std::vector<int> &path) {
	TArray<FVector> pathCoordinates;

	for (int i = 0; i < path.size(); i++) {
		pathCoordinates.Add(allPoints[path[i]]);
	}

	return pathCoordinates;
}


void AMapGen::print(FString msg, FColor color, float time) {
	GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
}



FVector AMapGen::getPoint(PolyPoint pp) {
	return allGroundPoints[pp.polygon_index][pp.point_index];
}



















