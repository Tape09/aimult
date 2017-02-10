// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRTFunctions.h"

TArray<TArray<FVector>> generatePoints(int nPoints, TArray<TArray<FVector>> bounds, TArray<TArray<FVector>> polygons) {

	TArray<FVector> RRTpoints;
	TArray<FVector> boundPoints;

	FVector tempPoint;
	float xmin = 100000000;// float_inf_;
	float ymin = 100000000;//float_inf_;
	float xmax = -100000000;//-float_inf_;
	float ymax = -100000000;//-float_inf_;
	for (int i = 0; i < bounds.Num(); i++) {
		tempPoint = bounds[i][0];
		boundPoints.Add(tempPoint);
		if (tempPoint.X <= xmin) xmin = tempPoint.X;
		if (tempPoint.Y <= ymin) ymin = tempPoint.Y;
		if (tempPoint.X >= xmax) xmax = tempPoint.X;
		if (tempPoint.Y >= ymax) ymax = tempPoint.Y;
	}

	//map->print("X: min=" + FString::SanitizeFloat(xmin) + " max=" + FString::SanitizeFloat(xmax));
	//map->print("Y: min=" + FString::SanitizeFloat(ymin) + " max=" + FString::SanitizeFloat(ymax));

	bool inBounds = false; //want to be true
	bool inPolygon = true; //want to be false
	tempPoint = FVector(0, 0, 0);// default_Z);
	int numSkippedPoints = 0;
	for (int i = 0; i < nPoints; i++) {

		inBounds = false;
		inPolygon = true;

		int s = 0;
		while (!inBounds || inPolygon) {
			tempPoint.X = FMath::FRandRange(xmin, xmax);
			tempPoint.Y = FMath::FRandRange(ymin, ymax);

			//in bounds?
			inBounds = isInPolygon(tempPoint, boundPoints);

			//in a polygon?
			for (int j = 0; j < polygons.Num() - 1; j++) {
				inPolygon = isInPolygon(tempPoint, polygons[j]);
			}

			s++;
			if (s > 10) {
				numSkippedPoints++;
				break;
			}
		}
		RRTpoints.Add(tempPoint);
		//DrawDebugPoint(GetWorld(), tempPoint + trace_offset, 2.5, FColor::Blue, true);
	}
	//print("Skipped " + FString::FromInt(numSkippedPoints) + " points of " + FString::FromInt(nPoints), FColor::Blue);

	TArray<TArray<FVector>> Return;
	Return.Add(RRTpoints);
	Return.Add(boundPoints);
	return Return;
}

bool isInPolygon(FVector point, TArray<FVector>polyBounds) {
	//returns true if point in polygon
	float angleSum = 0;
	for (int i = 0; i < polyBounds.Num() - 1; i++) {
		angleSum += getAngle(point - polyBounds[i], point - polyBounds[i + 1]);
	}
	angleSum += getAngle(point - polyBounds[0], point - polyBounds[polyBounds.Num() - 1]);

	if (abs(angleSum - 2.f*pi) < 0.001) return true;
	return false;
}

bool isInAnyPolygon(FVector tempPoint, TArray<TArray<FVector>> polygons) {
	bool inPolygon = false;
	for (int j = 0; j < polygons.Num() - 1; j++) {
		inPolygon = isInPolygon(tempPoint, polygons[j]);
		if (inPolygon)
			break;
	}
	return inPolygon;
}

FVector randVel(FString strategy, float max_v) {
	float vel;
	if (strategy == "max speed")			//Always max velocity! (random direction)
		vel = max_v;
	else if (strategy == "random speed")	//Random velocity! (random direction)	
		vel = FMath::FRandRange(0, max_v);
	else if (strategy == "random speed")	//Low velocity (random direction)
		vel = max_v / 2;

	float vx = FMath::FRandRange(0, vel);
	float vy = FMath::Sqrt(vel*vel - vx*vx);

	if (FMath::RandBool())
		vx = -vx;
	if (FMath::RandBool())
		vy = -vy;

	if (vx == 0 && vy == 0)
		print("zero vel :(", FColor::Red);
	return FVector(vx, vy, 0);
}