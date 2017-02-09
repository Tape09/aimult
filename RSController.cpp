// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RSController.h"


// Sets default values
ARSController::ARSController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.bAllowTickOnDedicatedServer = true; //need to add this for some reason??!?
	
}

// Called when the game starts or when spawned
void ARSController::BeginPlay()
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

	SetActorTickEnabled(true); //??!?!
}

// Called every frame
void ARSController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	time_to_init -= DeltaTime;
	if (time_to_init < 0 && !has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");
	}
}

void ARSController::init() {
	map->print("try RRT*");
	RRT = GetWorld()->SpawnActor<ARSRRT>();
	RRT->buildTree(map);
	has_initialized = false; //stop "ticking"

	//RRT = GetWorld()->SpawnActor<ARRT>();
	//RRT->buildTree(map, "SimpleCar");

	// JUST TESTING....

	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, map->goal_pos, map->goal_vel);
	/*FVector pos0(0, 0, 1);
	FVector pos1(5, 0, 1);
	FVector vel0(1, 0, 0);
	FVector vel1(-1, 0, 0);*/

	FVector pos0(30, 500, 0);
	FVector pos1(-300, 800, 0);
	FVector vel0(0, 100, 0);
	FVector vel1(0, -100, 0);


	//float vel;
	//float acc;
	//RSPaths rs = calc_path(map->start_pos, map->start_vel, map->goal_pos, map->goal_vel);
	//RSPaths rs = calc_path(pos0, vel0, pos1, vel1);
	//DrawDebugPoint(GetWorld(), pos0 + FVector(0, 0, 20), 7.5, FColor::Green, true);
	//DrawDebugPoint(GetWorld(), pos1 + FVector(0, 0, 20), 7.5, FColor::Green, true);
	map->print_log("p0: " + pos0.ToString());
	map->print_log("p1: " + pos1.ToString());


	map->print_log("v0: " + vel0.ToString());
	map->print_log("v1: " + vel1.ToString());

	map->print_log("diff: " + (pos1 - pos0).ToString());
	map->print_log("angle: " + FString::SanitizeFloat(wrapAngle(vecAngle(vel1) - vecAngle(vel0))));

	/*
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
	}*/


	//map->print_log("t2x: " + FString::SanitizeFloat(dp.path[0].t2));


}

// calculate path between two points and velocities
RSPaths ARSController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	RSPaths rs(pos0, vel0, pos1, vel1, map->v_max, map->phi_max, map->L_car);
	/*
	int bestPath_index = -1;
	float resolution = 100;
	float time;
	bool valid = true;
	//Path bestPath;
	for (int i = 0; i < rs.all_paths.size(); i++) {
		State s = rs.state_at(0, i);
		if (rs.path_time(i) <= 0)
			continue;
		valid = true;

		rs.reset();
		//check if path = valid
		for (int j = 0; j < resolution; j++) {
			if (bestPath_index != -1)
				break;
			time = j*rs.path_time(i) / resolution;
			s = rs.state_at(i, time);
			DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 50), 2.5, color, true);
			}
		}
		//if (valid) {
		if (FVector::Dist(s.vel, vel1) < 10 && FVector::Dist(s.pos, pos1) < 10 && valid) {

			print_log("pos: " + s.pos.ToString() + "       target pos: " + pos1.ToString());
			bestPath_index = i;
			rs.valid = true;
			break;
		}
		else
			rs.valid = false;
	}
	if (bestPath_index < 0)
		rs.valid = false;
	rs.path_index = bestPath_index;
	rs.reset();
	*/
	return rs;
}