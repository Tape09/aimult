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
	Node pathNode = dijkstras();
	path = map->getPath(pathNode.path);
	for (int i = 0; i < path.Num(); i++) {
		map->print(path[i].ToString());
		map->print_log(path[i].ToString());
		//GEngine->AddOnScreenDebugMessage(-1, 500.f, FColor::Cyan, Path[i].ToString());
	}
	drawPath();
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
