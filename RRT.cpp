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


void ARRT::buildTree(FVector start, FVector end)
{
	GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Build RRT");

	//GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, FString::SanitizeFloat(rand));
	
	
	RRTnode* currentNode = new RRTnode;
	currentNode->pos = start;
	currentNode->prev = NULL;

	RRTnode* newNode;
	bool goal_reached = false;

	std::queue<RRTnode*> Q;
	Q.push(currentNode);
	int stop = 0;

	while (!goal_reached) {

		currentNode = Q.front();
		Q.pop();

		if (currentNode->prev == NULL && currentNode->next == NULL) continue;

		newNode = newRRTnode(currentNode, 40);
		Q.push(newNode);
		Q.push(currentNode);


		stop++;
		if (stop > 1000) 
		{
			GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Could not find goal");
			break;
		}
	}

}

RRTnode* ARRT::newRRTnode(RRTnode* current, float r)
{
	FVector currPos = current->pos;

	float dx = randFloat(currPos.X - r, currPos.X + r);
	float dy = randFloat(currPos.Y - r, currPos.Y + r);
	FVector newPos = FVector(dx, dy, currPos.Z);
	
	bool traceOK = Trace(currPos, newPos, -1);
	int stop = 0;
	while (!traceOK) {
		dx = randFloat(currPos.X - r, currPos.X + r);
		dy = randFloat(currPos.Y - r, currPos.Y + r);
		newPos = FVector(dx, dy, currPos.Z);
		traceOK = Trace(currPos, newPos, -1);
		stop++;
		if (stop > 30) {
			GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Could not find point to trace to");

			RRTnode* empty = new RRTnode;
			empty->prev = NULL;
			empty->next = NULL;
			empty->pos = FVector(0,0,0);
			return empty;
		}
	}

	DrawDebugLine(GetWorld(), currPos + trace_offset, newPos + trace_offset, FColor::Yellow, true, -1.f, 30.f);

	RRTnode* traceTo = new RRTnode;
	traceTo->pos = newPos;
	traceTo->prev = current;

	current->next = traceTo;
	return traceTo;
}

float ARRT::randFloat(float min, float max)
{
	return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (min - max)));
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
		return true;
	}
	else {
		for (int i = 0; i < Hits.Num(); ++i) {
			float dist = Hits[i].Distance;
			if (dist == 0.0) continue;
			first_hit_dist = dist;
			break;
		}
	}

	if (first_hit_dist == 0.0) first_hit_dist = expected_dist;

	float dist_error = abs(first_hit_dist - expected_dist) / expected_dist;

	return dist_error < 0.1;
}