// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include "MapGen.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "RRT.generated.h"

struct RRTnode {

	RRTnode* prev; //previou nodes (parent)
	//TArray<RRTnode*> next;
	FVector pos;
	float dist_to_prev;

	//TODO: add:
	//FVector v; //speed/direction 
	//float probability; //higher closet to corners
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

	TArray<RRTnode*> RRT_tree;

	void generatePoints(int nPoints);

	RRTnode* findNearest(FVector pos, float max_dist);

	TArray<TArray<FVector>> polygons;
	TArray<TArray<FVector>> bounds;

	//TArray<int> visited;
	TArray<RRTnode*> inTree;
	TArray<FVector> notInTree;

	float default_Z;

private:
	//TODO: hämta från mapgen
	//float minX = -2000;
	//float maxX = 0;
	//float minY = 0;
	//float maxY = 2000;

	AMapGen* map;

	TArray<FVector> RRTpoints;

	const FVector trace_offset = FVector(0, 0, 50);
};
