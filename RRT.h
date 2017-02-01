// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <limits>
#include "MapGen.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "RRT.generated.h"

struct RRTnode {

	RRTnode* prev; //previou nodes (parent)
	FVector pos;
	float dist_to_prev;
	float tot_path_length;
	//TODO: add:
	//FVector velocity;
	//FVector acceleration;
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
	virtual void Tick( float DeltaSeconds ) override;

	void buildTree(AMapGen* map);

	bool Trace(FVector start, FVector end, int polyNum);

	bool isInPolygon(FVector point, TArray<FVector>polyBounds);

	float getAngle(FVector a, FVector b);

	void generatePoints(int nPoints);

	RRTnode* findNearest(FVector pos, float max_dist);

	TArray<TArray<FVector>> polygons;
	TArray<TArray<FVector>> bounds;
	TArray<FVector> boundPoints;

	float default_Z;

private:

	AMapGen* map;

	TArray<RRTnode*> inTree;
	TArray<FVector> notInTree;
		
	TArray<FVector> RRTpoints;
	TArray<RRTnode*> neighborhood;	//nodes in neighborhood
	float neighborhood_size = 200;	//size of neighborhood

	const FVector trace_offset = FVector(0, 0, 50);

	float float_inf = std::numeric_limits<float>::infinity();
};
