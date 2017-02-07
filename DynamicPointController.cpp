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

	fromFile = true; //use path from file
	fileName = "path";

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
		if (!goal_found && has_initialized && (!fromFile && RRTpath.Num() > 0 || fromFile && dpFromFile.Num()>0)) {
			if (J == 0) {
				if (fromFile)
					dp = dpFromFile[I];
				else
					dp = RRTpath[I]->dPath;
				dp.reset();
				currGoal = dp.final_pos();
				time = dp.path_time() / resolution;
				t = 0; //start ticking from 0
				s = dp.step(0);
				map->print("---------- start v " + s.vel.ToString());
				print_log("------------------------------------------------");
				result = result + "\n" + FString::SanitizeFloat(s.pos.X) + ", " + FString::SanitizeFloat(s.pos.Y) + ", " + FString::SanitizeFloat(s.vel.X) + ", " + FString::SanitizeFloat(s.vel.Y);
			}
			print_log(" v " + s.vel.ToString() + "   a " + s.acc.ToString() + "   p " + s.pos.ToString());
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

				if ((fromFile && I== dpFromFile.Num()-1) || (!fromFile && I== RRTpath.Num()-1)) {

					//final goal found!
					goal_found = true;
					map->print("Goal reached in " + FString::SanitizeFloat(t_tot));
					saveToFile();
				}
				I++;
				J = 0;
			}
			else
				J++;

			s = dp.step(DeltaTime);
		}
	}
	if (t > -1) {
		t += DeltaTime;
		t_tot += DeltaTime;
	}
	
}

void ADynamicPointController::init() {
	if (fromFile) {
		readFromFile();
		drawPath(dpFromFile, GetWorld());
	}
	else {
		RRT = GetWorld()->SpawnActor<ARRT>();
		RRTpath = RRT->buildTree(map, controller);
	}
}




void ADynamicPointController::saveToFile() {
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

void  ADynamicPointController::readFromFile() {
	////Read file
	FString csvFile = FPaths::GameDir() + "Data/" + fileName + ".csv";
	TArray<FString> take;
	FFileHelper::LoadANSITextFileToStrings(*(csvFile), NULL, take);
	FVector temp = FVector(0,0,0);

	print("take: " + FString::FromInt(take.Num()));
	for (int i = 0; i < take.Num(); i++)
	{
		FString aString = take[i];

		TArray<FString> stringArray = {};

		aString.ParseIntoArray(stringArray, TEXT(","), false);

		//print("stringArray: " + FString::FromInt(stringArray.Num()));
		if (stringArray.Num() == 4) {
			
			//positions
			temp.X = FCString::Atof(*stringArray[0]);
			temp.Y = FCString::Atof(*stringArray[1]);
			posArr.Add(temp);
			//velocitys
			temp.X = FCString::Atof(*stringArray[2]);
			temp.Y = FCString::Atof(*stringArray[3]);
			velArr.Add(temp);
		}

	}


	//Create dynamic paths
	for (int i = 0; i < posArr.Num()-1; i++) {
		DynamicPath dp(posArr[i], velArr[i], posArr[i+1], velArr[i+1], map->v_max, map->a_max);
		dpFromFile.Add(dp);
	}

	//add ens pos
	DynamicPath dp(posArr[posArr.Num()-1], velArr[velArr.Num() - 1], map->goal_pos, map->goal_vel, map->v_max, map->a_max);
	dpFromFile.Add(dp);
}



void ADynamicPointController::drawPath(TArray<DynamicPath> path, UWorld* world) {

	for (int j = 0; j < path.Num(); j++) {
		DynamicPath DP = path[j];

		float resolution = 100;
		float time = DP.path_time() / resolution;
		DP.reset();
		State s;
		for (int i = 0; i <= resolution; i++) {
			if (i == 0)
				s = DP.step(0);
			else
				s = DP.step(time);
			DrawDebugPoint(world, s.pos + FVector(0, 0, 10), 2.5, FColor::Yellow, true);
		}
	}

}