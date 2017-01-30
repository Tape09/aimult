// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"


// Sets default values
ARRT::ARRT()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ARRT::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void ARRT::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}


void ARRT::buildTree(AMapGen* map)
{
	int nPoints = 20;

	FVector start = map->start_pos;
	FVector end = map->goal_pos;
	default_Z = map->default_Z;

	GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Build RRT");

	polygons = map->allGroundPoints; //kom ih�g: sista polygonen i polygons = end_pos!
	bounds = map->allWallPoints;
	generatePoints(nPoints);
	map->print("points generated", 500.f, FColor::Magenta);

	notInTree = RRTpoints;
	notInTree.Add(end);

	RRTnode* start_node = new RRTnode;
	start_node->pos = start;
	start_node->prev = NULL;
	inTree.Add(start_node);

	int randIndex = 0;

	FVector tempPos1;
	FVector tempPos2;

	bool goal_reached = false;
	int iters = 0;
	int max_iters = nPoints+1; //problem_B --> v�ldigt l�ngsamt med m�nga iterationer (50 punkter & max_iters=50 redan l�ngsamt)

	while (!goal_reached) {
		// Pick random point, find nearest point in tree
		if (notInTree.Num() == 0) break;
		randIndex = FMath::RandRange(0, notInTree.Num() - 1);
		
		tempPos1 = notInTree[randIndex];
		RRTnode* node = findNearest(tempPos1, 1500);
		
		if (node->pos == FVector(NULL, NULL, NULL))
			continue;

		inTree.Add(node);
		notInTree.RemoveAt(randIndex, 1, true);

		if (tempPos1 == end) {
			map->print("Goal found!"); 
			goal_reached=true; //break or continue and find a better path?
		}
		iters++;
		if (iters > max_iters) {
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, "Could not find goal");
			break;
		}
	}

	//Traceback
	if (goal_reached) {
		RRTnode* node = inTree[inTree.Num() - 1];
		TArray<RRTnode*> path;
		while (node->prev != NULL) {
			path.Add(node);
			DrawDebugLine(GetWorld(), node->pos + trace_offset, node->prev->pos + trace_offset, FColor::Red, true, -1.f, 50.f);
			node = node->prev;
		}
	}
}


bool ARRT::Trace(FVector start, FVector end, int polyNum) {
	// polyNum = polygon number to be ignored 
	// or -1 if ignore none 

	FCollisionQueryParams QParams = FCollisionQueryParams();
	FCollisionResponseParams RParams = FCollisionResponseParams();
	QParams.bTraceComplex = true;

	TArray<FHitResult> Hits;

	//if (polyNum != -1) {
	//	QParams.AddIgnoredActor(allPolygons[polyNum]);
	//}

	GetWorld()->LineTraceMultiByChannel(Hits, start + trace_offset, end + trace_offset, ECollisionChannel::ECC_GameTraceChannel1, QParams, RParams);

	float expected_dist = FVector::Dist(start, end);
	float first_hit_dist = 0;

	if (Hits.Num() == 0) {
		//DrawDebugPoint(GetWorld(), end + trace_offset, 5.5, FColor::Green, true);
		return true;
	}
	else {
		for (int i = 0; i < Hits.Num(); ++i) {
			float dist = Hits[i].Distance;
			//DrawDebugPoint(GetWorld(), Hits[i].ImpactPoint, 5.5, FColor::Blue,true);
			if (dist == 0.0) continue;
			first_hit_dist = dist;
			break;
		}
	}

	if (first_hit_dist == 0.0) first_hit_dist = expected_dist;

	float dist_error = abs(first_hit_dist - expected_dist) / expected_dist;

	return dist_error == 0;// < 0.001 funkar ej!
}

void ARRT::generatePoints(int nPoints) {
	map->print("generate points");

	TArray<FVector> boundPoints;
	FVector tempPoint;
	float xmin = 1000000;
	float ymin = 1000000;
	float xmax = -1000000;
	float ymax = -1000000;
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
	tempPoint = FVector(0, 0, default_Z);
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
			//if(!inBounds) map->print("outside! " + tempPoint.ToString());

			//in a polygon?
			for (int j = 0; j < polygons.Num()-1; j++) {
				inPolygon = isInPolygon(tempPoint, polygons[j]);
			}
			//if (inPolygon) map->print("in polygon! " + tempPoint.ToString());

			s++;
			if (s > 10) {
				numSkippedPoints++;
				break;
			}
		}
		
		DrawDebugPoint(GetWorld(), tempPoint + trace_offset, 5.5, FColor::Blue, true);
		RRTpoints.Add(tempPoint);
	}
	map->print("Skipped " + FString::FromInt(numSkippedPoints) + " points of " + FString::FromInt(nPoints));
}


bool ARRT::isInPolygon(FVector point, TArray<FVector>polyBounds) {
	//returns true if point in polygon
	float angleSum = 0;
	for (int i = 0; i < polyBounds.Num()-1; i++) {
		angleSum+= getAngle(point-polyBounds[i], point - polyBounds[i+1]);
	}
	angleSum += getAngle(point - polyBounds[0], point - polyBounds[polyBounds.Num() - 1]);

	if (abs(angleSum - 2.f*3.14) < 0.01) return true; //TODO: use exact Pi
	return false;
}

float ARRT::getAngle(FVector a, FVector b) {
	float dot = a.X*b.X + a.Y*b.Y; // dot product
	float det = a.X*b.Y - a.Y*b.X; // determinant
	float angle = atan2(det, dot);
	return abs(angle);
}

RRTnode* ARRT::findNearest(FVector pos, float max_dist) {
	// Find nearest point in RRTpoints (returns index in RRTpoints)
	RRTnode* newNode = new RRTnode;
	newNode->pos = FVector(NULL, NULL, NULL);

	float nearestDist = max_dist; //only search within this range
	int nearest = -2;
	for (int i = 0; i < inTree.Num(); i++) {
		//check collision with inTree[i]
		if (FVector::Dist(pos, inTree[i]->pos) < nearestDist) {
			if (Trace(pos, inTree[i]->pos, -1)) {

				//if (FVector::Dist(pos, inTree[i]) < nearestDist) {
				nearest = i;
				nearestDist = FVector::Dist(pos, inTree[i]->pos);
				//}
			}
		}
	}
	if (nearest >= 0) {
		DrawDebugLine(GetWorld(), pos + trace_offset, inTree[nearest]->pos + trace_offset, FColor::Blue, true, -1.f, 30.f);
	}
	else
		return newNode;
	
	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	return newNode;
}