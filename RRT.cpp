// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"

//TODO:
// * optimize path: g� igenom v�gen och hoppa �ver on�diga noder
// * optimera alla v�gar och sen hitta den kortaste ist f�r tv�rt om?

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
	int nPoints = 1000;

	//Choose strategy
	strategy = "max speed";
	//strategy = "random speed";
	//strategy = "low speed";

	//Choose neighbourhood size
	if (controller_type == "DynamicPoint") {
		neighborhood_size = 3;		//measured in time
	}
	else if(controller_type == "KinematicPoint") {
		neighborhood_size = 200;	//measured in dist
	}

	bool optimize = false; //obs funkar ej med kinematic point och kanske inte annars heller...

	FVector start = map->start_pos;
	goal_pos = map->goal_pos;
	goal_vel = map->goal_vel;
	max_a = map->a_max;
	max_v = map->v_max;
	default_Z = map->default_Z;

	GEngine->AddOnScreenDebugMessage(0, 5.f, FColor::Yellow, "Build RRT");

	polygons = map->allGroundPoints;
	bounds = map->allWallPoints;
	generatePoints(nPoints);
	map->print("points generated", 500.f, FColor::Yellow);

	notInTree = RRTpoints;
	notInTree.Add(goal_pos);

	RRTnode* start_node = new RRTnode();
	start_node->pos = start;
	start_node->prev = NULL;
	start_node->tot_path_cost = 0;
	start_node->v = map->start_vel;
	start_node->dPath = DynamicPath();
	inTree.Add(start_node);

	//DrawDebugPoint(GetWorld(), start + FVector(0,0,100), 10.5, FColor::Red, true);
	//DrawDebugPoint(GetWorld(), goal_pos + FVector(0, 0, 100), 10.5, FColor::Red, true);

	int randIndex = 0;

	FVector tempPos1;

	RRTnode* goal = new RRTnode();
	goal->pos = FVector(NULL, NULL, NULL);
	goal->tot_path_cost = float_inf;

	bool goal_reached = false;
	int iters = 0;
	TArray<RRTnode*> goalNodes;//array with all nodes that reached the goal
	int max_iters = 2*nPoints;
	RRTnode* node;

	while (!goal_reached) {

		// Break if too many iterations or all points in tree
		if (iters > max_iters || notInTree.Num() == 1) {
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
		if (tempPos1 == goal_pos) {
			//om kortare v�g --> spara den
			map->print_log("goal cost/time: " + FString::SanitizeFloat(node->tot_path_cost) + " points left: " + FString::FromInt(notInTree.Num()));
			if (node->tot_path_cost < goal->tot_path_cost)
				goal = node;
		}
		else {
			inTree.Add(node);
			//DrawDebugLine(GetWorld(), tempPos1 + trace_offset, node->pos + trace_offset, FColor::Yellow, true, -1.f, 0, 2.5f);
			notInTree.RemoveAt(randIndex, 1, true);
		}
	}

	//Traceback from goal to start
	if (goal->pos != FVector(NULL, NULL, NULL)) {

		//Find shortest path
		map->print("goal found! :)  cost: " + FString::SanitizeFloat(goal->tot_path_cost));
		FVector trace_offset2 = FVector(0, 0, trace_offset.Z + 10.f);

		node = goal;


		
		//Optimise path!
		//takes forever even if stop?
		if (optimize) {
			////Draw not opt. path
			//int nNodes = 0;
			//while (node->prev != NULL) {
			//	if (controller_type == "DynamicPoint") {
			//		TArray<FVector> path_ = node->dPath2;

			//		//Draw dyn path
			//		for (int i = 0; i < path_.Num(); i++) {
			//			DrawDebugPoint(GetWorld(), path_[i] + trace_offset2, 2.5, FColor::Green, true);
			//		}
			//		//DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Yellow, true, -1.f, 0, 5.f);
			//	}
			//	else if(controller_type == "KinematicPoint") {
			//		DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Green, true, -1.f, 0, 10);
			//	}
			//	node = node->prev;
			//	nNodes++;
			//}
			//map->print(FString::FromInt(nNodes) + " number of nodes in path");

			node = goal;
			RRTnode* node2;
			float minCost = goal->tot_path_cost;
			int it = 0;
			while (node->prev->prev != NULL) {
				node2 = node;
				while (node2->prev->prev != NULL) {
					temp_dPath2.Empty();
					DynamicPath dp = calc_path(node2->prev->prev->pos, node2->prev->prev->v, node2->pos, node2->v);
					if (dp.valid && dp.path_time() < minCost) {
						node2->prev = node2->prev->prev;
						node2->tot_path_cost = node2->prev->tot_path_cost + dp.path_time();
						node2->dPath2 = temp_dPath2;
						node2->dPath = dp;
						minCost = dp.path_time();
					}
					node2 = node2->prev;

					it++;
					if (it > 500)
						break;
				}
				if (node->prev->prev != NULL)
					node = node->prev;
				else
					break;
				if (it > 500)
					break;
			}
		}
		
		node = goal;

		//Draw final path and add path to array
		TArray<RRTnode*> path;
		while (node->prev != NULL) {
			map->print("pos: " + node->pos.ToString() + "   vel: " + node->v.ToString());
			path.Add(node);

			if (controller_type == "DynamicPoint") {
				//Draw dyn path
				//TODO: draw path from node->dPath instead... funkade ej :( 
				TArray<FVector> path_ = node->dPath2;
				for (int i = 0; i < path_.Num(); i++) {
					DrawDebugPoint(GetWorld(), path_[i] + trace_offset2, 2.5, FColor::Red, true);
				}
				//DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Blue, true, -1.f, 0, 5.f);
			}
			else if (controller_type == "KinematicPoint") {
				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Red, true, -1.f, 0, 10);
			}
			node = node->prev;
		}
		Algo::Reverse(path);
		return path;
	}
	map->print("cound not find goal :(");
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
	float xmin = float_inf;
	float ymin = float_inf;
	float xmax = -float_inf;
	float ymax = -float_inf;
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
	map->print("Skipped " + FString::FromInt(numSkippedPoints) + " points of " + FString::FromInt(nPoints));
}

bool ARRT::isInPolygon(FVector point, TArray<FVector>polyBounds) {
	//returns true if point in polygon
	float angleSum = 0;
	for (int i = 0; i < polyBounds.Num() - 1; i++) {
		angleSum += getAngle(point - polyBounds[i], point - polyBounds[i + 1]);
	}
	angleSum += getAngle(point - polyBounds[0], point - polyBounds[polyBounds.Num() - 1]);

	if (abs(angleSum - 2.f*pi) < 0.001) return true; //TODO: use exact Pi
	return false;
}

float ARRT::getAngle(FVector a, FVector b) {
	float dot = a.X*b.X + a.Y*b.Y; // dot product
	float det = a.X*b.Y - a.Y*b.X; // determinant
	float angle = atan2(det, dot);
	return abs(angle);
}

RRTnode* ARRT::findNearest(FVector pos, float max_cost) {
	// Find nearest point in tree

	TArray<RRTnode*> neighborhood;
	RRTnode* newNode = new RRTnode();

	FVector v2;
	if (pos == goal_pos)
		v2 = goal_vel;

	float smallestCost = float_inf;
	float costToRootNode; //to minimize
	int nearest = -2;
	float vel;
	for (int i = 0; i < inTree.Num(); i++) {


		if (controller_type == "DynamicPoint") {
			temp_dPath2.Empty();

			if(strategy=="max speed")				//Always max velocity! (random direction)
				vel = max_v;
			else if (strategy == "random speed")	//Random velocity! (random direction)	
				vel = FMath::FRandRange(0, max_v); 
			else if (strategy == "random speed")	//Low velocity (random direction)
				vel = max_v / 2; 

			float vx = FMath::FRandRange(0, vel);
			float vy = FMath::Sqrt(vel*vel - vx*vx);

			if (pos != goal_pos)
				v2 = FVector(vx, vy, 0);

			DynamicPath dp = calc_path(inTree[i]->pos, inTree[i]->v, pos, v2);

			if (dp.valid) {

				if (dp.path_time() <= neighborhood_size)
					neighborhood.Add(inTree[i]);

				costToRootNode = inTree[i]->tot_path_cost + dp.path_time();
				if (costToRootNode < smallestCost) {
					if (temp_dPath2.Num() > 0) {
						nearest = i;
						smallestCost = costToRootNode;
						newNode->dPath = dp;
						newNode->dPath2 = temp_dPath2;
						newNode->cost_to_prev = dp.path_time();
					}
				}
			}
		}
		else if (controller_type == "KinematicPoint" && Trace(pos, inTree[i]->pos, -1)) {
			costToRootNode = inTree[i]->tot_path_cost + FVector::Dist(pos, inTree[i]->pos);
			if (FVector::Dist(pos, inTree[i]->pos) <= neighborhood_size)
				neighborhood.Add(inTree[i]);
			if (costToRootNode < smallestCost) {
				nearest = i;
				smallestCost = costToRootNode;
				newNode->cost_to_prev = FVector::Dist(pos, inTree[i]->pos);
			}
		}
	}
	if (nearest < 0)
		return newNode;

	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	newNode->tot_path_cost = smallestCost;
	newNode->v = v2;


	//RRT* - check in neighbourhood if better 
	float smallest_pathCost = newNode->tot_path_cost;
	for (int i = 0; i < neighborhood.Num(); i++) {

		if (controller_type == "KinematicPoint") {
			costToRootNode = neighborhood[i]->tot_path_cost + FVector::Dist(neighborhood[i]->pos, pos);
			if (costToRootNode < smallest_pathCost) {
				newNode->prev = neighborhood[i];
				newNode->tot_path_cost = costToRootNode;
				newNode->cost_to_prev = FVector::Dist(neighborhood[i]->pos, pos);
				smallest_pathCost = costToRootNode;
			}
		}
		else if (controller_type == "DynamicPoint") {
			temp_dPath2.Empty();
			DynamicPath dp = calc_path(neighborhood[i]->pos, neighborhood[i]->v, pos, v2);
			if (dp.valid && temp_dPath2.Num()>0) {
				costToRootNode = neighborhood[i]->tot_path_cost + dp.path_time();
				if (costToRootNode < smallest_pathCost) {
					newNode->prev = neighborhood[i];
					newNode->tot_path_cost = costToRootNode;
					newNode->dPath2 = temp_dPath2;
					newNode->dPath = dp;
					newNode->cost_to_prev = dp.path_time();
					smallest_pathCost = costToRootNode;
				}
			}
		}

	}

	//DrawDebugLine(GetWorld(), pos + trace_offset, newNode->prev->pos + trace_offset, FColor::Yellow, true, -1.f, 2.5f);
	return newNode;
}

// calculate path between two points and velocities
DynamicPath ARRT::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DynamicPath dp(pos0, vel0, pos1, vel1, max_v, max_a);

	// NEED TO CHEK HERE IF DP IS VALID. USE dp.state_at(time) TO GO THROUGH PATH AT SOME RESOLUTION (DT) AND CHECK IF INSIDE POLYGON. 
	// time VARIABLE IS RELATIVE TO THIS PATH, NOT ABSOLUTE TIME: 0 <= time <= dp.path_time()
	// USE dp.is_valid() after to check for path validity.

	if (dp.path_time() == 0 || !dp.exists) {
		dp.valid = false;
		return dp;
	}

	int resolution = 100;
	float time;
	time = dp.path_time() / resolution;
	dp.valid = true;
	FVector prevPos = FVector(NULL, NULL, NULL);
	for (int i = 0; i <= resolution-1; ++i) {

		//time = i * dp.path_time() / resolution;
		State s = dp.step(time);// dp.state_at(time);

		prevPos = s.pos;
		temp_dPath2.Add(s.pos);

		if (isInAnyPolygon(s.pos) || !isInPolygon(s.pos, boundPoints) ) {
			dp.valid = false;
			return dp;
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