// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "DynamicPointController.h"


// Sets default values
ADynamicPointController::ADynamicPointController()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void ADynamicPointController::BeginPlay()
{
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
void ADynamicPointController::Tick( float DeltaTime )
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

void ADynamicPointController::init() {
	
	// JUST TESTING....

	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, map->goal_pos, map->goal_vel);
	FVector pos0(-711,223,1);
	FVector pos1(-1000,0,1);
	FVector vel0(-141,7.5,1);
	FVector vel1(0,-200,1);

	DynamicPath dp = calc_path(pos0,vel0,pos1,vel1);

	map->print_log("p0: " + pos0.ToString());
	map->print_log("p1: " + pos1.ToString());
	map->print_log("px: " + dp.final_pos().ToString());

	map->print_log("v0: " + vel0.ToString());
	map->print_log("v1: " + vel1.ToString());
	map->print_log("vx: " + dp.final_vel().ToString());

	map->print_log("t1x: " + FString::SanitizeFloat(dp.path[0].t1));
	map->print_log("t2x: " + FString::SanitizeFloat(dp.path[0].t2));
	map->print_log("t3x: " + FString::SanitizeFloat(dp.path[0].t3));
	map->print_log("tx: " + FString::SanitizeFloat(dp.timex));

	map->print_log("t1y: " + FString::SanitizeFloat(dp.path[1].t1));
	map->print_log("t2y: " + FString::SanitizeFloat(dp.path[1].t2));
	map->print_log("t3y: " + FString::SanitizeFloat(dp.path[1].t3));
	map->print_log("ty: " + FString::SanitizeFloat(dp.timey));

}


// calculate path between two points and velocities
DynamicPath ADynamicPointController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath dp(pos0, vel0, pos1, vel1, map->v_max, map->a_max);

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

	return dp;
}