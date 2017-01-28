// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <algorithm>
#include <cstdlib>
#include <queue>
#include "DrawDebugHelpers.h"
#include "GameFramework/Actor.h"
#include "RRT.generated.h"

struct RRTnode {

	RRTnode* prev;
	RRTnode* next;
	FVector pos;
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

	void buildTree(FVector start, FVector end);

	RRTnode* newRRTnode(RRTnode* current, float r);

	float randFloat(float min, float max);

	bool Trace(FVector start, FVector end, int polyNum);

private:
	//TODO: hämta från mapgen
	float minX = -2000;
	float maxX = 0;
	float minY = 0;
	float maxY = 2000;

	const FVector trace_offset = FVector(0, 0, 50);
};
