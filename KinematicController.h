// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "RRT.h"
#include "DrawDebugHelpers.h"
#include "KinematicController.generated.h"

UCLASS()
class AIMULT_API AKinematicController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AKinematicController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	TArray<FVector> interpolate(FVector s, FVector t, float v);
	void ACar (FVector startPos, TArray<FVector> path);

	void drawPath();
	void init();
	Node dijkstras();



	float time_to_init = 1.5;
	bool has_initialized = false;

	TArray<FVector> path;
	TArray<RRTnode*> RRTpath;
	AMapGen * map;
	ARRT* RRT;

	TArray<FVector> currPath;
	FVector currGoal;
	FVector step = FVector(0, 0, 0);

	int I;
	int J;
	float speed;

	int type; //0 = visibility graph, 1 = rrt
	
};
