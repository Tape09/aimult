// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Json.h"
#include "Polygon.h"
#include "Car.h"
#include "Goal.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <cstdlib>
#include "DrawDebugHelpers.h"
#include "MapGen.generated.h"


struct PolyPoint {
	PolyPoint(int poly_idx, int point_idx) : polygon_index(poly_idx), point_index(point_idx) {}

	int polygon_index;
	int point_index;

	bool operator==(const PolyPoint & other) const {
		return (polygon_index == other.polygon_index) && (point_index == other.point_index);
	}
};

struct Node {
	
	double dist;
	std::vector<PolyPoint> path; // current point is path.back()

	bool operator<(const Node & other) const{
		return dist > other.dist;
	}
};

UCLASS()
class AIMULT_API AMapGen : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AMapGen();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void readJson(FString fileName);

	bool Trace(FVector start, FVector end, int ignorePolygon);

	TArray<FVector> getPath(std::vector<PolyPoint> &path);

	void print(FString msg, float time = 500.0, FColor color = FColor::Cyan);

	FVector getPoint(PolyPoint pp);

	void print_log(FString msg);

	void initFakeGroundPoints();

	void drawLine(FVector from, FVector to, FColor color = FColor(255,0,0), FVector z_offset = FVector(0,0,10));

	int Nvertices;

	float L_car;
	float a_max;
	float k_friction;
	float omega_max;
	float phi_max;
	float v_max;
	const float scale = 100;
	const float default_Z = 0;

	const FVector trace_offset = FVector(0,0,100);

	FVector goal_pos;
	FVector start_pos;
	FVector goal_vel;
	FVector start_vel;

	TArray<TArray<FVector>> allGroundPoints;
	TArray<TArray<FVector>> allFakeGroundPoints;


	TArray<TArray<FVector>> allWallPoints;
	TArray<FVector> allPoints;
	TArray<FVector> cornerPoints;
	TArray<FVector> wallPoints;

	TArray<APolygon *> allPolygons;
	TArray<APolygon *> allFakePolygons;
	TArray<APolygon *> allWalls;

	ACar * car;
	AGoal * goal;
};

