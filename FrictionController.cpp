// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "FrictionController.h"


// Sets default values
AFrictionController::AFrictionController() {
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AFrictionController::BeginPlay() {
	Super::BeginPlay();

	const UWorld * world = GetWorld();

	if (world) {
		FActorSpawnParameters spawnParams;
		spawnParams.Owner = this;
		spawnParams.Instigator = Instigator;

		map = GetWorld()->SpawnActor<AMapGen>(FVector(0, 0, 0), FRotator::ZeroRotator, spawnParams);
	}

	v_max = map->v_max;
	a_max = map->a_max;

	float k1 = tan(map->phi_max) * tan(map->phi_max) / pow(map->L_car / 100, 2.0);
	float k2 = pow(9.81, 2.0)*pow(map->k_friction, 2.0);

	float vm4 = pow(v_max / 100, 4.0);
	float am2 = pow(a_max / 100, 2.0);

	while ((vm4*k1 / k2 + am2 / k2) > 1) {
		v_max *= 0.99;
		a_max *= 0.99;

		k1 = tan(map->phi_max) * tan(map->phi_max) / pow(map->L_car / 100, 2.0);
		k2 = pow(9.81, 2.0)*pow(map->k_friction, 2.0);

		vm4 = pow(v_max / 100, 4.0);
		am2 = pow(a_max / 100, 2.0);
	}

	map->print("Map initializing...", 50);
	map->print_log("Map initializing...");

}

// Called every frame
void AFrictionController::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	if (!has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");
		t_now = 0;
		pidx = 0;

		if (my_path.size() > 0) {
			my_path[pidx]->path->reset();
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
		if (t_now > my_path[pidx]->path->path_time()) {
			dt = t_now - my_path[pidx]->path->path_time();
			if (pidx + 1 >= my_path.size()) {
				is_driving = false;
				map->car->SetActorLocationAndRotation(my_path[pidx]->path->pos_1(), my_path[pidx]->path->vel_1().Rotation());
				return;
			} else {
				++pidx;
				t_now = 0;
			}
		} else {
			dt = DeltaTime;
		}

		State s = my_path[pidx]->path->step(dt);

		map->car->SetActorLocationAndRotation(s.pos, s.vel.Rotation());
		DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 30), 2.5, FColor::Red, true);

	}

}

// calculate path between two points and velocities
std::shared_ptr<Path> AFrictionController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	FrictionCarPaths* dp = new FrictionCarPaths(pos0, vel0, pos1, vel1, v_max, map->phi_max, map->L_car, a_max);

	for (int j = 0; j < dp->n_paths(); ++j) {
		int resolution = 10 * dp->path_time(j);

		dp->valid = true;
		for (int i = 0; i <= resolution; ++i) {
			float time = i * dp->path_time(j) / resolution;
			State s = dp->state_at(j, time);
			if (isInAnyPolygon(s.pos, map->allGroundPoints) || !isInPolygon(s.pos, map->wallPoints)) {
				dp->valid = false;
				break;
			}
		}
		if (dp->isValid()) {
			dp->path_index = j;
			break;
		}
	}

	return std::shared_ptr<Path>(dp);
}




void AFrictionController::init() {
	RRT rrt(200, map, std::bind(&AFrictionController::calc_path, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), v_max, a_max);

	//file_log("asdf1");
	my_path = rrt.get_full_path();
	//file_log("asdf2");

	//for (int i = 0; i < rrt.nodes.size(); ++i) {
	//	DrawDebugPoint(GetWorld(), rrt.nodes[i]->pos + FVector(0, 0, 30), 5.0, FColor::Red, true);
	//}

	if (my_path.size() > 0) {
		for (int i = 0; i < my_path.size(); ++i) {

			print_log(my_path[i]->pos.ToString());
			print_log(FString::SanitizeFloat(my_path[i]->path->path_time()));
			print_log(FString::SanitizeFloat(my_path[i]->path->pathExists()));
			DrawDebugPoint(GetWorld(), my_path[i]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
		}

		print("TIME TAKEN: " + FString::SanitizeFloat(my_path.back()->cost));
	}
	//FVector pos0(-100,100,0);
	//FVector pos1(-400,1000,0);
	//FVector vel0(-100,0,0);
	//FVector vel1(0,100,0);

	//my_path.push_back(std::make_shared<RRTNode>(RRTNode(std::make_shared<RRTNode>(RRTNode(pos0,vel0)), calc_path(pos0, vel0, pos1, vel1),pos0)));

	//print_log("INIT DONE");
}

