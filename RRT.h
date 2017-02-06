// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <limits>

#include "myMath.h"
#include "MapGen.h"
#include "DynamicPath.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Algo/Reverse.h"
#include "RRT.generated.h"

struct RRTnode {

	RRTnode() {}
	~RRTnode() {}

	RRTnode* prev; //previous node (parent)
	FVector pos = FVector(NULL, NULL, NULL);
	float cost_to_prev; //cost = time or dist
	float tot_path_cost;
	FVector v = FVector(NULL, NULL, NULL);
	FVector a = FVector(NULL, NULL, NULL);
	DynamicPath dPath; //if controller type = dynamic... d = path between this and prev
	TArray<FVector> dPath2; //fullösning, kunde inte få ut vägen från dPath..
};


UCLASS()
class AIMULT_API ARRT : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ARRT();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	TArray<RRTnode*> buildTree(AMapGen* map, FString controller);

	bool Trace(FVector start, FVector end, int polyNum);

	bool isInPolygon(FVector point, TArray<FVector>polyBounds);

	void generatePoints(int nPoints);

	RRTnode* findNearest(FVector pos, float max_cost);

	DynamicPath calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);

	bool isInAnyPolygon(FVector tempPoint);

	FVector randVel();

	TArray<TArray<FVector>> polygons;
	TArray<TArray<FVector>> bounds;
	TArray<FVector> boundPoints;

private:
	AMapGen* map;
	FString controller_type;

	FVector goal_pos;
	FVector goal_vel;
	float max_a;
	float max_v;

	TArray<RRTnode*> inTree;
	TArray<FVector> notInTree;
	TArray<FVector> RRTpoints;
	TArray<FVector> temp_dPath2;
	float temp_dPath2_len;

	float neighborhood_size;	//size of neighborhood
	float neighborhood_max_cost;

	float dynPathLen; //används inte

	const FVector trace_offset = FVector(0, 0, 50);
	float float_inf = std::numeric_limits<float>::infinity();
	float default_Z;

	float pi = 3.141592653589793238463;

	FString strategy;


	//input
	UWorld * world;
	APlayerController * playerInput;
	bool opt;
};