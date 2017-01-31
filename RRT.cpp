// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"

//TODO:
// * stannar efter att ha hittat målet. Vill fortsätta och hitta BÄSTA vägen.
// * borde gå lite snabbare att söka igenom polygonerna om de är sorterade efter storleksordn. pga större sannolikhet att punkten ligger i en större polygon
// * path smoothing

// Sets default values
ARRT::ARRT()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	//PrimaryActorTick.bCanEverTick = true;
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
	int nPoints = 1000;

	FVector start = map->start_pos;
	FVector end = map->goal_pos;
	default_Z = map->default_Z;

	GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Build RRT");

	polygons = map->allGroundPoints; //kom ihåg: sista polygonen i polygons = end_pos!
	bounds = map->allWallPoints; 
	generatePoints( nPoints); 
	map->print("points generated", 500.f, FColor::Yellow);

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
	TArray<RRTnode*> goalNodes;//array with all nodes that reached the goal
	int max_iters = 2*nPoints + 1;

	while (!goal_reached) {

		// Break if too many iterations
		if (iters > max_iters) {
			GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, "Could not find goal");
			break;
		}
		iters++;

		// Pick random point, find nearest point in tree
		if (notInTree.Num() == 0) break;
		randIndex = FMath::RandRange(0, notInTree.Num() - 1);
		tempPos1 = notInTree[randIndex];

		RRTnode* node = findNearest(tempPos1, 500);

		if (node->pos == FVector(NULL, NULL, NULL)) {
			//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Yellow, "Could not add point to tree");
			continue;
		}

		if (tempPos1 == end) {
			//map->print("Goal found!"); 
			//goal_reached=true;
			goalNodes.Add(node);
		}
		else {
			inTree.Add(node);
			notInTree.RemoveAt(randIndex, 1, true);
		}
	}
	
	
	//Traceback
	bool b = false;
	FVector trace_offset2 = FVector(0, 0, trace_offset.Z + 5.f);
	if (goalNodes.Num()>0) {
		map->print(FString::FromInt(goalNodes.Num()) + " paths to the goal was found!");
		RRTnode* node = goalNodes[0];
		TArray<RRTnode*> path;
		while (node->prev != NULL) {
			b = false;
			if (node->prev->prev != NULL) {
				if (node->dist_to_prev + node->prev->dist_to_prev > FVector::Dist(node->pos, node->prev->prev->pos)) {
					//check if ok
					if (Trace(node->pos, node->prev->prev->pos, -1)) {
						b = true;
					}

				}
			}
			
			if (b) {
				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->prev->pos + trace_offset2, FColor::Red, true, -1.f, 0, 10);
				node = node->prev->prev;
			}
			else {
				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Red, true, -1.f, 0, 10);
				node = node->prev;
			}
			path.Add(node);

			//DrawDebugLine(GetWorld(), node->pos + trace_offset, node->prev->pos + trace_offset, FColor::Red, true, -1.f, 0, 10);
			//node = node->prev;
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
				if (inPolygon)
					break;
			}
			if (inPolygon)
				continue;
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

FVector ARRT::generatePoint(FVector point) {
	float diff = 100;
	float xmin = point.X - diff;
	float xmax = point.X + diff;
	float ymin = point.Y - diff;
	float ymax = point.Y + diff;

	bool inBounds = false; //want to be true
	bool inPolygon = true; //want to be false
	FVector tempPoint = FVector(0, 0, default_Z);

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
			if(inPolygon) continue;
		}
		//if (inPolygon) map->print("in polygon! " + tempPoit.ToString());

		s++;
		if (s > 10) {
			return FVector(NULL, NULL, NULL);
			break;
		}
	}
		
	//DrawDebugPoint(GetWorld(), tempPoint + trace_offset, 5.5, FColor::Blue, true);
	return tempPoint;
	//map->print("Skipped " + FString::FromInt(numSkippedPoints) + " points of " + FString::FromInt(nPoints));
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
		//DrawDebugLine(GetWorld(), pos + trace_offset, inTree[nearest]->pos + trace_offset, FColor::Yellow, true, -1.f, 30.f);
	}
	else
		return newNode;
	
	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	newNode->dist_to_prev = nearestDist;

	//check if better way
	if (inTree[nearest]->prev != NULL) {
		if (FVector::Dist(pos, inTree[nearest]->prev->pos) < nearestDist + inTree[nearest]->prev->dist_to_prev) {
			//check collision
			if (Trace(pos, inTree[nearest]->prev->pos, -1)) {
				newNode->prev = inTree[nearest]->prev;
				//DrawDebugLine(GetWorld(), pos + trace_offset, inTree[nearest]->pos + trace_offset, FColor::Blue, true, -1.f, 30.f);
			}
		}
	}

	DrawDebugLine(GetWorld(), pos + trace_offset, newNode->prev->pos + trace_offset, FColor::Blue, true, -1.f, 30.f);
	return newNode;
}