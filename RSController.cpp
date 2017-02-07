// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RSController.h"


// Sets default values
ARSController::ARSController()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
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

	result = "";
	fromFile = false; //use path from file
	fileName = "path";

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

		I = 0;
		J = 0;
	}

	//Follow path
	if (!goal_found && has_initialized && (!fromFile && RRTpath.Num() > 0 || fromFile && dpFromFile.Num()>0)) {
		if (J == 0) {
			if (fromFile)
				dp = dpFromFile[I];
			else
				dp = RRTpath[I]->rsPath;

			dp.reset();
			currGoal = dp.final_pos();
			time = dp.path_time(dp.path_index) / resolution;
			t = 0; //start ticking from 0
			s = dp.state_at(dp.path_index, 0);
			//map->print("---------- start v " + s.vel.ToString());
			//print_log("------------------------------------------------");
			result = result + "\n" + FString::SanitizeFloat(s.pos.X) + ", " + FString::SanitizeFloat(s.pos.Y) + ", " + FString::SanitizeFloat(s.vel.X) + ", " + FString::SanitizeFloat(s.vel.Y) + ", " + FString::SanitizeFloat(dp.path_index);
		}
		print_log(" v " + s.vel.ToString() + "   a " + s.acc.ToString() + "   p " + s.pos.ToString());
		//move car
		map->car->SetActorLocation(s.pos);

		//rotate car
		FVector dir = s.vel;
		dir.Normalize();
		FRotator rot = FRotator(0, dir.Rotation().Yaw, 0);
		map->car->SetActorRotation(rot);

		if (FVector::Dist(s.pos, currGoal) < 0.01) {
			map->print("---------- end v " + s.vel.ToString());
			map->print(FString::SanitizeFloat(t));

			//current goal found!
			map->print_log("Current goal " + FString::FromInt(I) + " reached");
			saveToFile();
			if ((fromFile && I == dpFromFile.Num() - 1) || (!fromFile && I == RRTpath.Num() - 1)) {

				//final goal found!
				goal_found = true;
				map->print("Goal reached in " + FString::SanitizeFloat(t_tot));
				
			}
			I++;
			J = 0;
		}
		else
			J++;

		//s = dp.step(DeltaTime);
		time = J*dp.path_time(dp.path_index) / resolution;
		s = dp.state_at(dp.path_index, time);
	}
	if (t > -1) {
		t += DeltaTime;
		t_tot += DeltaTime;
	}

}

// calculate path between two points and velocities
RSPaths ARSController::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	RSPaths rs(pos0, vel0, pos1, vel1, map->v_max, map->phi_max, map->L_car);

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

	int bestPath_index = -1;
	float resolution = 100;
	float time;
	for (int i = 0; i < rs.all_paths.size(); i++) {
		State s = rs.state_at(0, i);
	
		if (rs.path_time(i) != 0) {
			//check if path = valid
			print("draw path");
			rs.reset();
			for (int j = 0; j < resolution; j++) {

				time = j*rs.path_time(i) / resolution;
				s = rs.state_at(dp.path_index, time);
				DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 20), 2.5, FColor::Red, true);
			}
			break;
		}
	}
	//rs.path_index = bestPath_index;
	
	return rs;
}



void ARSController::init() {
	
	RRT = GetWorld()->SpawnActor<ARRT>();
	RRTpath = RRTpath = RRT->buildTree(map, "SimpleCar");

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


void ARSController::saveToFile() {
	FString SaveDirectory = FString(FPaths::GameDir() + "Data");
	FString FileName = FString("path.csv");
	bool AllowOverwriting = true;

	IPlatformFile& PlatformFile = FPlatformFileManager::Get().GetPlatformFile();
	if (PlatformFile.CreateDirectoryTree(*SaveDirectory))
	{
		FString AbsoluteFilePath = SaveDirectory + "/" + FileName;

		if (AllowOverwriting || !PlatformFile.FileExists(*AbsoluteFilePath))
		{
			FFileHelper::SaveStringToFile(result, *AbsoluteFilePath);
		}
	}
}

void  ARSController::readFromFile() {
	////Read file
	FString csvFile = FPaths::GameDir() + "Data/" + fileName + ".csv";
	TArray<FString> take;
	FFileHelper::LoadANSITextFileToStrings(*(csvFile), NULL, take);
	FVector temp = FVector(0, 0, 0);
	TArray<int> indArr;

	print("take: " + FString::FromInt(take.Num()));
	for (int i = 0; i < take.Num(); i++)
	{
		FString aString = take[i];

		TArray<FString> stringArray = {};

		aString.ParseIntoArray(stringArray, TEXT(","), false);

		//print("stringArray: " + FString::FromInt(stringArray.Num()));
		if (stringArray.Num() == 5) {

			//positions
			temp.X = FCString::Atof(*stringArray[0]);
			temp.Y = FCString::Atof(*stringArray[1]);
			posArr.Add(temp);
			//velocitys
			temp.X = FCString::Atof(*stringArray[2]);
			temp.Y = FCString::Atof(*stringArray[3]);
			velArr.Add(temp);

			//float ind = 
			int ind = FCString::Atoi(*stringArray[4]);
			indArr.Add(ind);
		}

	}


	//Create dynamic paths
	for (int i = 0; i < posArr.Num() - 1; i++) {
		RSPaths dp(posArr[i], velArr[i], posArr[i + 1], velArr[i + 1], map->v_max, map->phi_max, map->L_car);
		dp.path_index = indArr[i];
		dpFromFile.Add(dp);
	}

	//add ens pos
	RSPaths dp(posArr[posArr.Num() - 1], velArr[velArr.Num() - 1], map->goal_pos, map->goal_vel, map->v_max, map->phi_max, map->L_car);
	dp.path_index = 0; //?????????????????+
	dpFromFile.Add(dp);
}



void ARSController::drawPath(TArray<RSPaths> path, UWorld* world) {

	for (int j = 0; j < path.Num(); j++) {
		RSPaths DP = path[j];

		float resolution = 100;
		float time;// = DP.path_time(DP.path_index) / resolution;
		DP.reset();
		State s;
		for (int i = 0; i <= resolution; i++) {
			//if (i == 0)
			//	s = DP.step(0);
			//else
			//	s = DP.step(time);

			time = i*DP.path_time(i) / resolution;
			s = DP.state_at(DP.path_index, time);

			DrawDebugPoint(world, s.pos + FVector(0, 0, 10), 2.5, FColor::Yellow, true);
		}
	}
}