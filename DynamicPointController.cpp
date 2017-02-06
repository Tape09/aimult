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
				t = 0; //start ticking from 0
				s = dp.step(0);
				map->print("---------- start v " + s.vel.ToString());
			}
			resolution = 1000;
			//if (J <= resolution) {
				
				//s = dp.step(DeltaTime);

				//move car
				map->car->SetActorLocation(s.pos);

				//rotate car
				FVector dir = s.vel;
				dir.Normalize();
				FRotator rot = FRotator(0, dir.Rotation().Yaw, 0);
				map->car->SetActorRotation(rot);

				if (FVector::Dist(s.pos, currGoal) < 0.001) {
					map->print("---------- end v " + s.vel.ToString());
					map->print(FString::SanitizeFloat(t));

					//current goal found!
					map->print_log("Current goal " + FString::FromInt(I) + " reached");

					if (I== RRTpath.Num()-1) {

						//final goal found!
						goal_found = true;
						map->print("Goal reached in " + FString::SanitizeFloat(t_tot));
					}
					I++;
					J = 0;
				}
				else
					J++;

				s = dp.step(DeltaTime);
			//}
		}
	}
	if (t > -1) {
		t += DeltaTime;
		t_tot += DeltaTime;
	}
}

void ADynamicPointController::init() {
	RRT = GetWorld()->SpawnActor<ARRT>();
	RRTpath = RRT->buildTree(map, controller);
}




