// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "KinematicController.h"


// Sets default values
AKinematicController::AKinematicController()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AKinematicController::BeginPlay()
{
	Super::BeginPlay();

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		map = GetWorld()->SpawnActor<AMapGen>(FVector(0, 0, 0), FRotator::ZeroRotator, spawnParams);
	}

	//FPlatformProcess::Sleep(3);

	//map->print(FString::FromInt(map->allGroundPoints.Num()));
	//map->print(map->start_pos.ToString());

	


	map->print("Map initializing...", 50);
	map->print_log("Map initializing...");
}

void AKinematicController::init() {
	type = 1; //1 = RRT
	//type = 0; //0 = visibility graph

	if (type == 0) {
		Node pathNode = dijkstras();
		path = map->getPath(pathNode.path);
		for (int i = 0; i < path.Num(); i++) {
			map->print(path[i].ToString());
			map->print_log(path[i].ToString());
			//GEngine->AddOnScreenDebugMessage(-1, 500.f, FColor::Red, path[i].ToString());
		}
		drawPath();
	}

	if (type == 1) {
		//RRT (not for kinematic point & not finished)
		ARRT* RRT = GetWorld()->SpawnActor<ARRT>();
		RRT->buildTree(map);
	}

}


// Called every frame
void AKinematicController::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	
	time_to_init -= DeltaTime;
	if (time_to_init < 0 && !has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");

		if (type == 0) {
			I = 1; //start at path[1] since path[0] is current position
			currGoal = path[I];

			currPath = interpolate(map->start_pos, currGoal, 100);
		}
	}

	//Follow path
	if (has_initialized && type == 0) {
		FVector loc = map->car->GetActorLocation();
		map->car->SetActorLocation(path[0]); //--> crash

		if (J + 1 == currPath.Num()) {
			//current goal reached!

			if (I + 1 == path.Num()) {
				//goal reached!
				GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Green, "Goal reached in x seconds");
			}
			else {
				//set next goal
				I++;
				currGoal = path[I];
				currPath = interpolate(loc, currGoal, 100);
			}
		}
		else {
			J++;
			map->car->SetActorLocation(currPath[J]);
		}
	}

}

Node AKinematicController::dijkstras() {

	std::priority_queue<Node> Q;

	Node node;
	int ignorePolygon = -1;

	node.dist = 0;
	node.path.push_back(PolyPoint(-1, 0)); // -1,0 is start, -1,1 is goal
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

		//map->print(FString::FromInt(map->allGroundPoints.Num()));

		// loop over polygons
		for (int i = 0; i < map->allGroundPoints.Num(); ++i) {
			// loop over points in polygon
			for (int j = 0; j < map->allGroundPoints[i].Num(); ++j) {
				// if point not in current path

				if (std::find(node.path.begin(), node.path.end(), PolyPoint(i, j)) == node.path.end()) {
					// if same polygon
					ignorePolygon = -1;
					if (node.path.back().polygon_index == i && (map->allGroundPoints.Num() - 1 != i)) {
						// set ignore polygon i
						ignorePolygon = i;

						if (node.path.back().point_index != (j - 1) % map->allGroundPoints[i].Num() && node.path.back().point_index != (j + 1) % map->allGroundPoints[i].Num()) continue;
					}


					FVector start = map->getPoint(node.path.back());
					FVector end = map->allGroundPoints[i][j];
					bool free = map->Trace(start, end, ignorePolygon);

					map->print_log("TRACE:: ");
					map->print_log("FROM:: " + start.ToString());
					map->print_log("TO:: " + end.ToString());
					map->print_log("SUCCESS:: " + FString::FromInt(free));
					map->print_log("==============");


					if (free) {
						map->drawLine(start, end, FColor::Blue, FVector(0, 0, 1));

						node2.dist = FVector::Dist(start, end) + node.dist;
						node2.path = node.path;
						node2.path.push_back(PolyPoint(i, j));

						if (map->getPoint(PolyPoint(i, j)) == map->goal_pos) {
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



void AKinematicController::drawPath() {
	
	for (int i = 0; i < path.Num() - 1; ++i) {
		map->drawLine(path[i],path[i+1]);
	}

}


TArray<FVector> AKinematicController::interpolate(FVector s, FVector t, float v)
{
	TArray<FVector> positions;

	int steps = 10 * FVector::Dist(s, t) / v;

	FVector Step = (t - s) / steps;
	FVector prev = s;

	for (int i = 0; i < steps; i++)
	{
		FVector New = prev + FVector(Step.X, Step.Y, 0);
		positions.Add(New);
		prev = New;
	}
	J = 0;
	return positions;
}
