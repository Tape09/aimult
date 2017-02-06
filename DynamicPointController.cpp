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

	controller = "DynamicPoint";
	//controller = "KinematicPoint";
}

// Called every frame
void ADynamicPointController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	time_to_init -= DeltaTime;
	if (time_to_init < 0 && !has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");

		//init follow path
		I = 0;
		J = 0;
	}

	//Follow path
	if (controller == "DynamicPoint") {
		if (!goal_found && has_initialized && RRTpath.Num() > 0) {	
			if (J == 0) {
				dp = RRTpath[I]->dPath;
				dp.reset();
				currGoal = dp.final_pos();
				time = dp.path_time() / resolution;
			}

			if (J <= resolution) {
				s = dp.step(time);

				//move car
				map->car->SetActorLocation(s.pos);

				//rotate car
				FVector dir = s.vel;
				dir.Normalize();
				FRotator rot = FRotator(0, dir.Rotation().Yaw, 0);
				map->car->SetActorRotation(rot);

				if (s.pos == currGoal) {

					//current goal found!
					map->print_log("Current goal " + FString::FromInt(I) + " reached");

					if (I== RRTpath.Num()-1) {

						//final goal found!
						goal_found = true;
					}
					I++;
					J = 0;
				}
				else
					J++;
			}
		}
	}
}

void ADynamicPointController::init() {

	// JUST TESTING....
	RRT = GetWorld()->SpawnActor<ARRT>();
	RRTpath = RRT->buildTree(map, controller);

	/*

	
	FVector v1 = map->start_pos;
	FVector v2 = map->start_pos + map->start_vel;
	
	FVector goalVel = map->goal_vel;
	//goalVel = FVector(FMath::FRandRange(0, -map->v_max), FMath::FRandRange(0, map->v_max), 0); //test random vel
	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, map->goal_pos, goalVel);
	//DynamicPath dp = calc_path(map->goal_pos, goalVel, map->start_pos, map->start_vel);
	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, map->goal_pos, goalVel);

	//test different  points
	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, FVector(-400,600,0), goalVel);
	//DynamicPath dp = calc_path(map->start_pos, map->start_vel, FVector(-400, 600, 0), FVector(100,100,0));
	DynamicPath dp = calc_path(map->start_pos, map->start_vel, FVector(-400, 400, 0), FVector(100, 100, 0));

	//v1 = FVector(-21, 93, 0);
	//v2 = FVector(0, -2000, 0);
	//DynamicPath dp = calc_path(FVector(-846,110,0), v1, FVector(-1000, 0, 0), v2);
	//DrawDebugPoint(GetWorld(), FVector(-846, 110, 0) + FVector(0, 0, 10), 10.5, FColor::Red, true);
	//DrawDebugPoint(GetWorld(), FVector(-1000, 0, 0) + FVector(0, 0, 10), 10.5, FColor::Red, true);



	v1.Z = 10;
	v2.Z = 10;
	DrawDebugLine(GetWorld(), v1, v2, FColor::Blue, true, -1.f, 30.f);
	v1 = map->goal_pos;
	v2 = map->goal_pos + goalVel;// map->goal_vel;
	v1.Z = 10;
	v2.Z = 10;
	DrawDebugLine(GetWorld(), v1, v2, FColor::Blue, true, -1.f, 30.f);

	map->print_log("p0: " + map->start_pos.ToString());
	map->print_log("p1: " + map->goal_pos.ToString());
	map->print_log("px: " + dp.final_pos().ToString());

	map->print_log("v0: " + map->start_vel.ToString());
	map->print_log("v1: " + map->goal_vel.ToString());
	map->print_log("vx: " + dp.final_vel().ToString());

	map->print_log("t1x: " + FString::SanitizeFloat(dp.path[0].t1));
	map->print_log("t2x: " + FString::SanitizeFloat(dp.path[0].t2));
	map->print_log("t3x: " + FString::SanitizeFloat(dp.path[0].t3));

	map->print_log("t1y: " + FString::SanitizeFloat(dp.path[1].t1));
	map->print_log("t2y: " + FString::SanitizeFloat(dp.path[1].t2));
	map->print_log("t3y: " + FString::SanitizeFloat(dp.path[1].t3));
	*/
}




