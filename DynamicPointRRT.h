// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <limits>
#include "MyMath.h"
#include "RRTFunctions.h"
#include "MapGen.h"
#include "Car.h"
#include "DynamicPath.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Algo/Reverse.h"
#include "DynamicPointRRT.generated.h"

struct DynRRTnode {

	DynRRTnode() {}
	~DynRRTnode() {}

	DynRRTnode* prev; //previous node (parent)
	FVector pos = FVector(NULL, NULL, NULL);
	float tot_path_cost;
	FVector v = FVector(NULL, NULL, NULL);
	Path* p;

	DynamicPath dPath;
};


UCLASS()
class AIMULT_API ADynamicPointRRT : public AActor
{
	GENERATED_BODY()

public:
	void buildTree(AMapGen* map);
private:
	// Sets default values for this actor's properties
	ADynamicPointRRT();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	//void buildTree(AMapGen* map);

	DynRRTnode* findNearest(FVector pos);

	DynamicPath calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);

	TArray<DynRRTnode*> drawPath(DynRRTnode* last_node, bool savePath, FColor color);

	TArray<TArray<FVector>> polygons;
	TArray<TArray<FVector>> bounds;
	TArray<FVector> boundPoints;
	float time_ = 0;

	AMapGen* map;
	FString controller_type;

	FVector goal_pos;
	FVector goal_vel;
	float max_a;
	float max_v;
	float max_phi;
	float car_L;

	bool goal_found;

	DynRRTnode* node;

	int count = 0;

	TArray<DynRRTnode*> path;

	TArray<DynRRTnode*> inTree;
	TArray<FVector> notInTree;
	TArray<FVector> RRTpoints;

	float neighborhood_size;	//size of neighborhood

	const FVector trace_offset = FVector(0, 0, 50);
	float float_inf = std::numeric_limits<float>::infinity();
	float default_Z;

	//float pi = 3.141592653589793238463;

	FString strategy;

	ACar* the_car;
	DynamicPath the_dp;
	Path* the_path;
};