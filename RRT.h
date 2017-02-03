// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <limits>
#include "MapGen.h"
#include "DynamicPath.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "Algo/Reverse.h"
#include "RRT.generated.h"

struct RRTnode {

	RRTnode* prev; //previous node (parent)
	FVector pos;
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

	float getAngle(FVector a, FVector b);

	void generatePoints(int nPoints);

	RRTnode* findNearest(FVector pos, float max_dist);

	DynamicPath calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);

	bool isInAnyPolygon(FVector tempPoint);

	TArray<TArray<FVector>> polygons;
	TArray<TArray<FVector>> bounds;
	TArray<FVector> boundPoints;

	float default_Z;

private:

	FVector goal_pos;
	FVector goal_vel;

	FString controller_type;
	AMapGen* map;

	TArray<RRTnode*> inTree;
	TArray<FVector> notInTree;

	TArray<FVector> RRTpoints;
	TArray<RRTnode*> neighborhood;	//nodes in neighborhood
	float neighborhood_size = 200;	//size of neighborhood

	const FVector trace_offset = FVector(0, 0, 50);

	float float_inf = std::numeric_limits<float>::infinity();

	float max_a;
	float max_v;

	float dynPathLen;

	TArray<FVector> temp_dPath2;
};