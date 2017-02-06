// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include <cmath>


const float pi = 4*FGenericPlatformMath::Atan(1);
const float twopi = 8 * FGenericPlatformMath::Atan(1);
float wrapAngle(float angle);
float vecAngle(const FVector & fv);
float rad2deg(const float & rad);
float mod2pi(float angle);


float mmod(float m, float n);



void rotateVector(FVector & fv, float theta);

void print_log(FString msg);





