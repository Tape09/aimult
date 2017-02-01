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
	DynamicPath dp(map->start_pos,map->start_vel,map->goal_pos,map->goal_vel,map->v_max,map->a_max);

	map->print_log("p0: " + map->start_pos.ToString());
	map->print_log("p1: " + map->goal_pos.ToString());
	map->print_log("px: " + dp.final_pos().ToString());

	map->print_log("v0: " + map->start_vel.ToString());
	map->print_log("v1: " + map->goal_vel.ToString());
	map->print_log("vx: " + dp.final_vel().ToString());

	map->print_log("t1x: " + FString::SanitizeFloat(dp.t_1[0]));
	map->print_log("t2x: " + FString::SanitizeFloat(dp.t_2[0]));
	map->print_log("t3x: " + FString::SanitizeFloat(dp.t_3[0]));

	map->print_log("t1y: " + FString::SanitizeFloat(dp.t_1[1]));
	map->print_log("t2y: " + FString::SanitizeFloat(dp.t_2[1]));
	map->print_log("t3y: " + FString::SanitizeFloat(dp.t_3[1]));



	
	


}

bool ADynamicPointController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	// X coordinate
	




	return true;
}