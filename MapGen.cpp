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
	print("MapGen!", FColor::Red, 20);

	Super::BeginPlay();

	FString problem = FString("problem_B");
	readJson(problem);
	initFakeGroundPoints();

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		for (int j = 0; j < allGroundPoints.Num(); ++j) {

			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allGroundPoints[j][0], FRotator::ZeroRotator, spawnParams);
			allPolygons.Add(newActor);
			newActor->init(allGroundPoints[j], FVector(0, 0, 0));

			newActor = GetWorld()->SpawnActor<APolygon>(allFakeGroundPoints[j][0], FRotator::ZeroRotator, spawnParams);
			allFakePolygons.Add(newActor);
			newActor->init(allFakeGroundPoints[j], FVector(0, 0, 0));

		}

		for (int j = 0; j < allWallPoints.Num(); ++j) {
			APolygon * newActor = GetWorld()->SpawnActor<APolygon>(allWallPoints[j][0], FRotator::ZeroRotator, spawnParams);
			newActor->init(allWallPoints[j], FVector(0, 0, 0));
			allWalls.Add(newActor);
		}
	}

	
	Node pathNode = dijkstras();
	TArray<FVector> Path = getPath(pathNode.path);
	for (int i = 0; i < Path.Num(); i++) {
		print(Path[i].ToString());
		print_log(Path[i].ToString());
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

	TArray<FVector> endPolygon;
	endPolygon.Add(goal_pos);
	allGroundPoints.Add(endPolygon);


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


	std::priority_queue<Node> Q;

	Node node;
	int ignorePolygon = -1;
	
	node.dist = 0;
	node.path.push_back(PolyPoint(-1,0)); // -1,0 is start, -1,1 is goal
	Q.push(node);


	int asdf = 0;

	Node node2;
	while (!Q.empty()) {
		if (asdf > 1000) {
			break;
		}
		asdf++;

		node = Q.top();
		Q.pop();

		// loop over polygons
		for (int i = 0; i < allGroundPoints.Num(); ++i) {
			// loop over points in polygon
			for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
				// if point not in current path
				
				if (std::find(node.path.begin(), node.path.end(), PolyPoint(i, j)) == node.path.end()) {
					// if same polygon
					ignorePolygon = -1;
					if (node.path.back().polygon_index == i && (allGroundPoints.Num()-1 != i)) {
						// set ignore polygon i
						ignorePolygon = i;

						if (node.path.back().point_index != (j - 1) % allGroundPoints[i].Num() && node.path.back().point_index != (j + 1) % allGroundPoints[i].Num()) continue;
					}


					FVector start = getPoint(node.path.back());
					FVector end = allGroundPoints[i][j];
					bool free = Trace(start, end, ignorePolygon);					

					if (free) {
						node2.dist = FVector::Dist(start,end) + node.dist;
						node2.path = node.path;
						node2.path.push_back(PolyPoint(i,j));

						if (getPoint(PolyPoint(i,j)) == goal_pos) {
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

bool AMapGen::Trace(FVector start, FVector end, int polyNum) {
	// polyNum = polygon number to be ignored 
	// or -1 if ignore none 

	//FCollisionQueryParams QParams(FName(TEXT("")), true, this);
	//QParams.bTraceAsyncScene = true;
	//QParams.bReturnPhysicalMaterial = false;
	//QParams.bTraceComplex = true;
	//FCollisionObjectQueryParams ObjQParams = FCollisionObjectQueryParams::AllStaticObjects;
	FCollisionQueryParams QParams = FCollisionQueryParams();
	FCollisionResponseParams RParams = FCollisionResponseParams();
	QParams.bTraceComplex = true;

	TArray<FHitResult> Hits;

	if (polyNum != -1) {
		QParams.AddIgnoredActor(allPolygons[polyNum]);
	}

	GetWorld()->LineTraceMultiByChannel(Hits, start+trace_offset, end+trace_offset, ECollisionChannel::ECC_GameTraceChannel1, QParams, RParams);

	//print_log(FString("SCAN FROM ") + start.ToString());
	//print_log(FString("SCAN TO ") + end.ToString());
	//print_log(FString::FromInt(Hits.Num()));
	//for (int i = 0; i < Hits.Num(); ++i) {
	//	print_log(Hits[i].ImpactPoint.ToString() + " " + FString::FromInt(Hits[i].IsValidBlockingHit()) + " " + Hits[i].GetActor()->GetName());
	//}
	//print_log("____");

	float expected_dist = FVector::Dist(start, end);
	float first_hit_dist = 0;

	if (Hits.Num() == 0) {
		return true;
	} else {
		for (int i = 0; i < Hits.Num(); ++i) {
			float dist = Hits[i].Distance;
			if(dist == 0.0) continue;
			first_hit_dist = dist;
			break;
		}
	}

	if(first_hit_dist == 0.0) first_hit_dist = expected_dist;

	float dist_error = abs(first_hit_dist - expected_dist) / expected_dist;

	return dist_error < 0.1;


	
}

TArray<FVector> AMapGen::getPath(std::vector<PolyPoint> &path) {
	TArray<FVector> pathCoordinates;

	for (int i = 0; i < path.size(); i++) {
		pathCoordinates.Add(getPoint(path[i]));
	}

	return pathCoordinates;
}


void AMapGen::print(FString msg, FColor color, float time) {
	GEngine->AddOnScreenDebugMessage(-1, time, color, msg);
}



FVector AMapGen::getPoint(PolyPoint pp) {

	if (pp == PolyPoint(-1, 0)) {
		return start_pos;
	} else if (pp == PolyPoint(-1, 1)) {
		return goal_pos;
	} else {
		return allGroundPoints[pp.polygon_index][pp.point_index];
	}

	
}

void AMapGen::print_log(FString msg) {
	UE_LOG(LogTemp, Warning, TEXT("%s"),*msg);
}

void AMapGen::initFakeGroundPoints() {
	allFakeGroundPoints = allGroundPoints;

	for (int i = 0; i < allGroundPoints.Num(); ++i) {

		FVector midpoint(0,0,0);
		for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
			midpoint = midpoint + allGroundPoints[i][j];
		}
		midpoint = midpoint / allGroundPoints[i].Num();

		for (int j = 0; j < allGroundPoints[i].Num(); ++j) {
			allFakeGroundPoints[i][j] = allGroundPoints[i][j] + (midpoint - allGroundPoints[i][j]) * 0.01;
		}
	}
}

















