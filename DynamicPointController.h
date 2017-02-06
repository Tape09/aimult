// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "DynamicPath.h"
#include "RRT.h"
#include "DynamicPointController.generated.h"

//struct DynamicPathX {
//	DynamicPathX(FVector pos0, FVector vel0, FVector pos1, FVector vel1)
//
//
//};



UCLASS()
class AIMULT_API ADynamicPointController : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ADynamicPointController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	void init();

	float time_to_init = 1.5;
	bool has_initialized = false;

	DynamicPath dp;
	float time;
	State s;
	float resolution = 100;
	float t = -1;
	float t_tot = 0;

	AMapGen * map;

	ARRT* RRT;
	TArray<RRTnode*> RRTpath;

	bool goal_found = false;
	FVector currGoal;
	TArray<FVector> currPath;
	int I;
	int J;

	FString controller;

};
