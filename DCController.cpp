// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "DCController.h"


// Sets default values
ADCController::ADCController() {
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADCController::BeginPlay() {
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

// Called every frame
void ADCController::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);

	time_to_init -= DeltaTime;
	if (time_to_init < 0 && !has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");
	}

}

// calculate path between two points and velocities
DynamicCarPaths ADCController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicCarPaths rs(pos0, vel0, pos1, vel1, map->v_max, map->phi_max, map->L_car, map->a_max);

	// NEED TO CHEK HERE IF DP IS VALID. USE dp.state_at(time) TO GO THROUGH PATH AT SOME RESOLUTION (DT) AND CHECK IF INSIDE POLYGON. 
	// time VARIABLE IS RELATIVE TO THIS PATH, NOT ABSOLUTE TIME: 0 <= time <= dp.path_time()
	// USE dp.is_valid() after to check for path validity.

	//dp.valid = true;
	//for (int i = 0; i <= resolution; ++i) {
	//	time = i * dp.path_time()/resolution;
	//	State s = dp.state_at(time);
	//	if (s.pos is inside a polygon) {
	//		dp.valid = false;
	//		break;
	//	}
	//}

	return rs;
}




void ADCController::init() {

	// JUST TESTING....

	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, map->goal_pos, map->goal_vel);
	/*FVector pos0(0, 0, 1);
	FVector pos1(5, 0, 1);
	FVector vel0(1, 0, 0);
	FVector vel1(-1, 0, 0);*/

	FVector pos0(0, 0, 0);
	FVector pos1(500, 0, 0);
	FVector vel0(0, 1, 0);
	FVector vel1(0, -1, 0);


	//float vel;
	//float acc;

	DynamicCarPaths rs = calc_path(pos0, vel0, pos1, vel1);

	map->print_log("p0: " + pos0.ToString());
	map->print_log("p1: " + pos1.ToString());


	map->print_log("v0: " + vel0.ToString());
	map->print_log("v1: " + vel1.ToString());

	map->print_log("diff: " + (pos1 - pos0).ToString());
	map->print_log("angle: " + FString::SanitizeFloat(wrapAngle(vecAngle(vel1) - vecAngle(vel0))));

	print_log(FString::SanitizeFloat(rs.all_paths.size()));
	for (int j = 0; j<rs.all_paths.size(); ++j) {
		//for(int j = 8; j<9; ++j) {
		State s = rs.state_at(j, rs.all_paths[j].time);
		
		//print_log(rs.all_paths[j].word());
		if ((s.pos - pos1).Size() > 0.1) {
			print_log(rs.all_paths[j].word());
			//for (int i = 0; i < rs.all_paths[j].size(); ++i) {
			//	map->print_log(rs.all_paths[j].components[i].toString());
			//}
			//print_log(FString::FromInt(j));
			//print_log(s);
			map->print_log(FString("========================"));
		}



	}


	//map->print_log("t2x: " + FString::SanitizeFloat(dp.path[0].t2));


}

