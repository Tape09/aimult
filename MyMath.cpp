// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "MyMath.h"

float wrapAngle(float angle) {
	angle = mod2pi(angle);
	return angle >= pi ? angle - 2 * pi : angle;
}

float vecAngle(const FVector & fv) {
	return wrapAngle(std::atan2(fv.Y, fv.X));
}

float mmod(float m, float n) {
	return m - n * floor(m / n);
}

float rad2deg(const float & rad) {
	return rad * 180 / pi;
}

float mod2pi(float angle) {
	while (angle < 0) angle += twopi;
	while (angle >= twopi) angle -= twopi;
	return angle;
}

void rotateVector(FVector & fv, float theta) {
	fv = fv.RotateAngleAxis(rad2deg(theta), FVector(0, 0, 1));
}

void print_log(FString msg) {
	UE_LOG(LogTemp, Warning, TEXT("%s"), *msg);
}

float getAngle(FVector a, FVector b) {
	float dot = a.X*b.X + a.Y*b.Y; 
	float det = a.X*b.Y - a.Y*b.X; 
	float angle = atan2(det, dot);
	return abs(angle);
}