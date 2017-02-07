// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "RRT.h"
#include "Car.h"
#include "DifferentialDrivePaths.h"
#include "MyMath.h"
#include "DDController.generated.h"

UCLASS()
class AIMULT_API ADDController : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ADDController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;


	void init();

	float time_to_init = 1.5;
	bool has_initialized = false;

	DifferentialDrivePaths calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);


	AMapGen * map;

	ARRT* RRT;
	TArray<RRTnode*> RRTpath;
};