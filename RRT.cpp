// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"

//TODO:
// * optimize path: gå igenom vägen och hoppa över onödiga noder
// * optimera alla vägar och sen hitta den kortaste ist för tvärt om?
// * lägg till goal_vel i slutnoden

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
void ARRT::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}


TArray<RRTnode*> ARRT::buildTree(AMapGen* map, FString controller)
{
	controller_type = controller;

	int nPoints = 100;

	FVector start = map->start_pos;
	FVector end = map->goal_pos;
	max_a = map->a_max;
	max_v = map->v_max;
	default_Z = map->default_Z;

	GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Build RRT");

	polygons = map->allGroundPoints; //kom ihåg: sista polygonen i polygons = end_pos!
	bounds = map->allWallPoints;
	generatePoints(nPoints);
	map->print("points generated", 500.f, FColor::Yellow);

	notInTree = RRTpoints;
	notInTree.Add(end);

	RRTnode* start_node = new RRTnode;
	start_node->pos = start;
	start_node->prev = NULL;
	start_node->tot_path_cost = 0;
	start_node->v = map->start_vel;
	start_node->dPath = DynamicPath(); //funkar?
	inTree.Add(start_node);

	int randIndex = 0;

	FVector tempPos1;
	FVector tempPos2;

	bool goal_reached = false;
	int iters = 0;
	TArray<RRTnode*> goalNodes;//array with all nodes that reached the goal
	int max_iters = 2 * nPoints;
	RRTnode* node;
	while (!goal_reached) {

		// Break if too many iterations
		if (iters > max_iters || notInTree.Num() == 0) {
			break;
		}
		iters++;

		// Pick random point, find nearest point in tree
		randIndex = FMath::RandRange(0, notInTree.Num() - 1);
		tempPos1 = notInTree[randIndex];


		node = findNearest(tempPos1, 500);

		if (node->pos == FVector(NULL, NULL, NULL)) {
			continue;
		}

		if (tempPos1 == end) {
			goalNodes.Add(node);
		}
		else {
			inTree.Add(node);
			notInTree.RemoveAt(randIndex, 1, true);
		}
	}



	//Traceback
	if (goalNodes.Num()>0) {

		//Find shortest path
		map->print(FString::FromInt(goalNodes.Num()) + " paths to the goal was found!");
		FVector trace_offset2 = FVector(0, 0, trace_offset.Z + 10.f);
		float minPathLength = float_inf;
		RRTnode* node = goalNodes[0];
		for (int i = 0; i < goalNodes.Num(); i++) {
			if (goalNodes[i]->tot_path_cost < minPathLength) {
				node = goalNodes[i];
				minPathLength = node->tot_path_cost;
			}
		}

		TArray<RRTnode*> path;
		while (node->prev != NULL) {

			//TODO: optimize path
			path.Add(node);
			if (controller_type == "DynamicPoint") {
				TArray<FVector> path_ = node->dPath2;

				//Draw dyn path
				for (int i = 0; i < path_.Num(); i++) {
					DrawDebugPoint(GetWorld(), path_[i] + trace_offset2, 2.5, FColor::Red, true);
				}
				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Yellow, true, -1.f, 0, 5.f);
				//if(path_.Num()>0)
				//	DrawDebugLine(GetWorld(), node->pos + trace_offset2, path_[0] + trace_offset2, FColor::Green, true, -1.f, 0, 5.f);
				//DrawDebugLine(GetWorld(), path_[0] + trace_offset2, node->prev->pos + trace_offset2, FColor::Green, true, -1.f, 0, 5.f);
			}
			else {
				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Red, true, -1.f, 0, 10);
			}
			node = node->prev;
		}
		Algo::Reverse(path);
		return path;
	}
	TArray<RRTnode*> empty;
	return empty;
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
			for (int j = 0; j < polygons.Num() - 1; j++) {
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

bool ARRT::isInPolygon(FVector point, TArray<FVector>polyBounds) {
	//returns true if point in polygon
	float angleSum = 0;
	for (int i = 0; i < polyBounds.Num() - 1; i++) {
		angleSum += getAngle(point - polyBounds[i], point - polyBounds[i + 1]);
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
	neighborhood.Empty();
	RRTnode* newNode = new RRTnode;
	newNode->pos = FVector(NULL, NULL, NULL);
	FVector v2 = FVector(FMath::FRandRange(0, -max_v), FMath::FRandRange(0, max_v), 0); //random vel

	float smallestCost = float_inf;
	float costToTreeNode;
	int nearest = -2;
	for (int i = 0; i < inTree.Num(); i++) {

		costToTreeNode = FVector::Dist(pos, inTree[i]->pos);

		if (costToTreeNode <= neighborhood_size) //TODO: add different for cost = time vs dist!
			neighborhood.Add(inTree[i]);

		if (controller_type == "DynamicPoint") {
			temp_dPath2.Empty();
			dynPathLen = 0; //används ej
			//DynamicPath dp = calc_path(pos, v2, inTree[i]->pos, inTree[i]->v);
			DynamicPath dp = calc_path(pos, v2, inTree[i]->pos, inTree[i]->v); //hänger dig om man byetr ordning??
			//TODO: try different v2:s here
			
			if (dp.valid && dp.path_time() < smallestCost) {
				if (temp_dPath2[0] != temp_dPath2[temp_dPath2.Num() - 1] && dynPathLen != 0) {
					nearest = i;
					smallestCost = dp.path_time();// dynPathLen;
					newNode->dPath = dp;
					newNode->dPath2 = temp_dPath2;
				}
			}
		}
		else if (controller_type != "DynamicPoint" && Trace(pos, inTree[i]->pos, -1) && costToTreeNode < smallestCost) {
			nearest = i;
			smallestCost = FVector::Dist(pos, inTree[i]->pos);
		}
	}
	if (nearest < 0)
		return newNode;

	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	newNode->cost_to_prev = smallestCost;
	newNode->tot_path_cost = newNode->prev->tot_path_cost + smallestCost;
	newNode->v = v2;

	// Check if better way in neighbourhood. TODO: do this for dyn path too
	if (controller_type == "RRT") {
		float pathCost;
		float smallest_pathCost = newNode->tot_path_cost;
		TArray<FVector> temp;
		for (int i = 0; i < neighborhood.Num(); i++) {
			if (controller_type == "RRT")
				pathCost = neighborhood[i]->tot_path_cost + FVector::Dist(neighborhood[i]->pos, pos); //have to check if ok?
			if (controller_type == "DynamicPoint") {
				
				temp_dPath2.Empty();
				dynPathLen = 0; //används ej
				DynamicPath dp = calc_path(pos, v2, neighborhood[i]->pos, neighborhood[i]->v);
				
				//TODO: try different v2:s here
				
				if (dp.valid && dp.path_time() < smallestCost) {
					if (temp_dPath2[0] != temp_dPath2[temp_dPath2.Num() - 1] && dynPathLen != 0) {
						temp = temp_dPath2;
						pathCost = neighborhood[i]->tot_path_cost + dp.path_time();
					}
					else
						continue;
				}
				else
					continue;
			}

			if (pathCost < smallest_pathCost) {
				newNode->prev = neighborhood[i];
				newNode->tot_path_cost = pathCost;
				smallest_pathCost = pathCost;
				newNode->dPath2 = temp;
			}
		}
	}

	//DrawDebugLine(GetWorld(), pos + trace_offset, newNode->prev->pos + trace_offset, FColor::Blue, true, -1.f, 30.f);
	if (newNode->prev != NULL) {
		newNode->tot_path_cost = newNode->prev->tot_path_cost + newNode->cost_to_prev;
	}

	return newNode;
}

// calculate path between two points and velocities
DynamicPath ARRT::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath dp(pos0, vel0, pos1, vel1, max_v, max_a);

	// NEED TO CHEK HERE IF DP IS VALID. USE dp.state_at(time) TO GO THROUGH PATH AT SOME RESOLUTION (DT) AND CHECK IF INSIDE POLYGON. 
	// time VARIABLE IS RELATIVE TO THIS PATH, NOT ABSOLUTE TIME: 0 <= time <= dp.path_time()
	// USE dp.is_valid() after to check for path validity.

	int resolution = 1000;
	float time;
	time = dp.path_time() / resolution;
	dp.valid = true;
	FVector prevPos = FVector(NULL, NULL, NULL);
	for (int i = 0; i <= resolution; ++i) {
		//time = i * dp.path_time() / resolution;
		State s = dp.step(time);// dp.state_at(time);

		if (prevPos != FVector(NULL, NULL, NULL)) {
			dynPathLen += FVector::Dist(s.pos, prevPos);
			temp_dPath2.Add(s.pos);
		}
		prevPos = s.pos;
		if (isInAnyPolygon(s.pos) || !isInPolygon(s.pos, boundPoints)) {
			dp.valid = false;
			break;
		}
	}

	return dp;
}

bool ARRT::isInAnyPolygon(FVector tempPoint) {
	bool inPolygon = false;
	for (int j = 0; j < polygons.Num() - 1; j++) {
		inPolygon = isInPolygon(tempPoint, polygons[j]);
		if (inPolygon)
			break;
	}
	return inPolygon;
}