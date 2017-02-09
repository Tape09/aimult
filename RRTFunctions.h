// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <string>
#include "MapGen.h"
#include "MyMath.h"

TArray<TArray<FVector>> generatePoints(int nPoints, TArray<TArray<FVector>> bounds, TArray<TArray<FVector>> polygons);
bool isInPolygon(FVector point, TArray<FVector>polyBounds);
bool isInAnyPolygon(FVector tempPoint, TArray<TArray<FVector>> polygons);
FVector randVel(FString strategy, float max_v);
