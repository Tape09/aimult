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

	v_max = map->v_max / sqrt(2);
	a_max = map->a_max / sqrt(2);

	map->print("Map initializing...", 50);
	map->print_log("Map initializing...");
}

// Called every frame
void ADynamicPointController::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );
	if (!has_initialized) {
		init();
		has_initialized = true;
		map->print("Map initialized!", 50);
		map->print_log("Map initialized!");
		t_now = 0;
		pidx = 0;
		if (my_path.size() > 0) {
			is_driving = true;
		}		
	}

	if (buffer_ticks > 0) {
		--buffer_ticks;
		return;
	}

	if(is_driving) {
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
	
		map->car->SetActorLocationAndRotation(s.pos,s.vel.Rotation());
		DrawDebugPoint(GetWorld(),s.pos + FVector(0,0,30),2.5,FColor::Red,true);

	}
}

void ADynamicPointController::init() {
	
	// JUST TESTING....

	RRT rrt(1000,map,std::bind(&ADynamicPointController::calc_path,this,std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),v_max,a_max);

	my_path = rrt.get_full_path();

	for (int i = 0; i < my_path.size(); ++i) {
		print_log(my_path[i]->pos.ToString());
		print_log(FString::SanitizeFloat(my_path[i]->path->path_time()));
		print_log(FString::SanitizeFloat(my_path[i]->path->pathExists()));
		DrawDebugPoint(GetWorld(), my_path[i]->pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
	}
	DrawDebugPoint(GetWorld(), map->start_pos + FVector(0, 0, 30), 15, FColor::Magenta, true);
	print("TIME TAKEN: " + FString::SanitizeFloat(my_path.back()->cost));



	//FVector point(-1509.732, 1188.879,100);
	//print_log(FString::FromInt(isInAnyPolygon(point,map->allGroundPoints)));

}


// calculate path between two points and velocities
std::shared_ptr<Path> ADynamicPointController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath* dp = new DynamicPath(pos0, vel0, pos1, vel1, v_max,  a_max);
	
	int resolution = 10 * dp->path_time();
	//int resolution = 50;
	dp->valid = true;
	for (int i = 0; i <= resolution; ++i) {
		float time = i * dp->path_time() / resolution;
		State s = dp->state_at(time);

		if ( isInAnyPolygon(s.pos, map->allGroundPoints) || !isInPolygon(s.pos, map->wallPoints)) {
			dp->valid = false;
			break;
		}
	}

	return std::shared_ptr<Path>(dp);
}