// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "KinematicController.h"


// Sets default values
AKinematicController::AKinematicController() {
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AKinematicController::BeginPlay() {
	Super::BeginPlay();

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		map = GetWorld()->SpawnActor<AMapGen>(FVector(0, 0, 0), FRotator::ZeroRotator, spawnParams);
	}


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
void AKinematicController::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);

	time_to_init -= DeltaTime;
	if (time_to_init < 0 && !has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");

		t_now = 0;
		pidx = 0;

		if (path.Num() > 0) {
			is_driving = true;
		}
	}

	if (buffer_ticks > 0) {
		--buffer_ticks;
		return;
	}

	if (is_driving) {
		float dt;
		t_now += DeltaTime;
		if (t_now > my_path_times[pidx]) {
			dt = t_now - my_path_times[pidx];
			if (pidx + 1 >= path.Num()) {
				is_driving = false;

				map->car->SetActorLocation(path[pidx]);
				return;
			} else {
				++pidx;
				t_now = 0;
				
			}
		} else {
			dt = DeltaTime;
		}

		if (pidx + 1 < path.Num()) {



			FVector dt_step = (path[pidx + 1] - path[pidx]) / my_path_times[pidx];
			dt_step.Z = 0;

			newPos = newPos + dt*dt_step;

			map->car->SetActorLocation(newPos);
			DrawDebugPoint(GetWorld(), newPos + FVector(0, 0, 30), 2.5, FColor::Red, true);
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

	float path_time = 0;
	for (int i = 0; i < path.Num() - 1; ++i) {
		//map->drawLine(path[i],path[i+1]);
		float temp_time = FVector::Dist(path[i], path[i + 1]) / map->v_max; //Always max speed
		path_time += temp_time;
		my_path_times.push_back(temp_time);
		DrawDebugPoint(GetWorld(), path[i] + FVector(0, 0, 30), 15, FColor::Magenta, true);
	}
	DrawDebugPoint(GetWorld(), map->goal_pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
	print_log("Path time: " + FString::SanitizeFloat(path_time));
	newPos = path[0];

}
