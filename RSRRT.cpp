// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RSRRT.h"

//TODO:
// * optimize path: gå igenom vägen och hoppa över onödiga noder
// * optimera alla vägar och sen hitta den kortaste ist för tvärt om?

// Sets default values
ARSRRT::ARSRRT()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ARSRRT::BeginPlay()
{
	Super::BeginPlay();

}

// Called every frame
void ARSRRT::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (goal_found) {
		if (time_ == 0) {
			the_dp = path[count]->dPath;
			the_dp.reset();
		}

		State s = the_dp.state_at(the_dp.path_index, time_);
		the_car->SetActorLocation(s.pos);
		time_ += DeltaTime;

		if (time_ >= the_dp.path_time(the_dp.path_index)) {
			count++;
			time_ = 0;
			if (count > path.Num() - 1)
				goal_found = false;
		}
	}
}

void ARSRRT::buildTree(AMapGen* map)
{
	print(" --- Simple Car RRT* ---", FColor::Red);
	int nPoints = 500;
	int max_iters = nPoints;

	//Choose strategy
	strategy = "max speed";
	//strategy = "random speed";
	//strategy = "low speed";

	//Choose neighbourhood size
	neighborhood_size = 3;	//measured in time

	FVector start = map->start_pos;
	goal_pos = map->goal_pos;
	goal_vel = map->goal_vel;
	max_a = map->a_max;
	max_v = map->v_max;
	max_phi = map->phi_max;
	car_L = map->L_car;
	default_Z = map->default_Z;

	polygons = map->allGroundPoints;
	bounds = map->allWallPoints;

	TArray<TArray<FVector>> r = generatePoints(nPoints, bounds, polygons);
	RRTpoints = r[0];
	boundPoints = r[1];

	print("points generated", FColor::Blue);

	notInTree = RRTpoints;
	notInTree.Add(goal_pos);

	rsRRTnode* start_node = new rsRRTnode();
	start_node->pos = start;
	start_node->prev = NULL;
	start_node->tot_path_cost = 0;
	inTree.Add(start_node);

	//DrawDebugPoint(GetWorld(), start + FVector(0,0,100), 10.5, FColor::Red, true);
	//DrawDebugPoint(GetWorld(), goal_pos + FVector(0, 0, 100), 10.5, FColor::Red, true);

	int randIndex = 0;

	FVector tempPos1;

	rsRRTnode* goal = new rsRRTnode();
	goal->pos = FVector(NULL, NULL, NULL);
	goal->tot_path_cost = float_inf;

	bool goal_reached = false;
	int iters = 0;
	TArray<rsRRTnode*> goalNodes;//array with all nodes that reached the goal

	while (!goal_reached) {

		// Break if too many iterations or all points in tree
		if (iters > max_iters || notInTree.Num() == 1) {
			break;
		}
		iters++;

		// Pick random point, find nearest point in tree
		randIndex = FMath::RandRange(0, notInTree.Num() - 1);
		tempPos1 = notInTree[randIndex];
		node = findNearest(tempPos1);

		//if (node->pos == FVector(NULL, NULL, NULL)) {
		if (node == NULL) {
			continue;
		}
		if (tempPos1 == goal_pos) {
			//om kortare väg --> spara den
			print_log("goal cost/time: " + FString::SanitizeFloat(node->tot_path_cost) + " points left: " + FString::FromInt(notInTree.Num()));
			if (node->tot_path_cost < goal->tot_path_cost) {
				goal = node;
				break; //ok fist time goal is found since it takes so long
			}
		}
		else {
			inTree.Add(node);
			//DrawDebugLine(GetWorld(), tempPos1 + trace_offset, node->pos + trace_offset, FColor::Yellow, true, -1.f, 0, 2.5f);
			notInTree.RemoveAt(randIndex, 1, true);
		}
	}

	//Traceback from goal to start
	if (goal->pos != FVector(NULL, NULL, NULL)) {

		print("goal found! :)  cost: " + FString::SanitizeFloat(goal->tot_path_cost), FColor::Blue);

		node = goal;
		path = drawPath(node, true, FColor::Red);
		Algo::Reverse(path);

		//start the car movement
		the_car = map->car;
		goal_found = true;
	}
	else
		print("cound not find goal :(", FColor::Blue);
}

rsRRTnode* ARSRRT::findNearest(FVector pos) {
	// Find nearest point in tree

	TArray<rsRRTnode*> neighborhood;
	rsRRTnode* newNode = new rsRRTnode();

	FVector v2;
	if (pos == goal_pos)
		v2 = goal_vel;

	float smallestCost = float_inf;
	float costToRootNode; //to minimize
	int nearest = -2;
	for (int i = 0; i < inTree.Num(); i++) {

		if (pos != goal_pos)
			v2 = randVel(strategy, max_v);

		float cost;
		bool valid = false;

		RSPaths dp;
		dp = calc_path(inTree[i]->pos, inTree[i]->v, pos, v2);
		cost = dp.path_time(dp.path_index);
		valid = dp.valid;

		if (valid) {

			if (cost <= neighborhood_size)
				neighborhood.Add(inTree[i]);

			costToRootNode = inTree[i]->tot_path_cost + cost;
			if (costToRootNode < smallestCost) {
				nearest = i;
				smallestCost = costToRootNode;
				newNode->v = v2;

				newNode->dPath = dp;
				newNode->p = &newNode->dPath;
			}
		}
	}
	if (nearest < 0)
		return newNode;

	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	newNode->tot_path_cost = smallestCost;


	//RRT* - check in neighbourhood if better 
	float smallest_pathCost = newNode->tot_path_cost;
	for (int i = 0; i < neighborhood.Num(); i++) {

		bool valid;
		float cost;

		RSPaths dp;
		dp = calc_path(neighborhood[i]->pos, neighborhood[i]->v, pos, v2);
		valid = dp.valid;
		cost = dp.path_time(dp.path_index);
		if (valid) {
			costToRootNode = neighborhood[i]->tot_path_cost + cost;
			if (costToRootNode < smallest_pathCost) {
				newNode->prev = neighborhood[i];
				newNode->tot_path_cost = costToRootNode;
				smallest_pathCost = costToRootNode;

				newNode->dPath = dp;
				newNode->p = &newNode->dPath;
			}
		}
	}

	//DrawDebugLine(GetWorld(), pos + trace_offset, newNode->prev->pos + trace_offset, FColor::Yellow, true, -1.f, 2.5f);
	return newNode;
}

// calculate path between two points and velocities
RSPaths ARSRRT::calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	RSPaths dp(pos0, vel0, pos1, vel1, max_v, max_phi, car_L);

	/*if (dp.path_time() == 0 || !dp.exists) {
		dp.valid = false;
		return dp;
	}

	int resolution = 100;
	float time;
	time = dp.path_time() / resolution;
	dp.valid = true;
	for (int i = 0; i <= resolution - 1; ++i) {

		State s = dp.step(time);
		if (isInAnyPolygon(s.pos, polygons) || !isInPolygon(s.pos, boundPoints)) {
			dp.valid = false;
			return dp;
		}
	}
	return dp;*/



	RSPaths rs(pos0, vel0, pos1, vel1, max_v, max_phi, car_L);

	int bestPath_index = -1;
	float resolution = 100;
	float time;
	bool valid = true;
	//Path bestPath;
	for (int i = 0; i < rs.all_paths.size(); i++) {
		State s = rs.state_at(0, i);
		if (dp.path_time(i) <= 0)
			continue;
		valid = true;

		rs.reset();
		//check if path = valid
		for (int j = 0; j < resolution; j++) {
			if (bestPath_index != -1)
				break;
			time = j*rs.path_time(i) / resolution;
			s = rs.state_at(i, time);

			if (isInAnyPolygon(s.pos, polygons) || !isInPolygon(s.pos, boundPoints)) {
				valid = false;
				// not valid
				break;
			}
		}
		if (valid) {//FVector::Dist(s.vel, vel1) < 0.001 && FVector::Dist(s.pos, pos1) < 0.001 && valid) {
			bestPath_index = i;
			rs.valid = true;
			break;
		}
	}
	if (bestPath_index < 0)
		rs.valid = false;
	rs.path_index = bestPath_index;
	rs.reset();
	return rs;
}


/* Step through path from end to start and draw it. And add nodes to path. */
TArray<rsRRTnode*> ARSRRT::drawPath(rsRRTnode* last_node, bool savePath, FColor color) {
	TArray<rsRRTnode*> pat;
	while (last_node->prev != NULL) {
		if (savePath)
			pat.Add(last_node);

		RSPaths path_ = last_node->dPath;
		path_.reset();
		float d_time = path_.path_time(path_.path_index) / 100;

		for (int i = 0; i <= 100; i++) {
			State s = path_.state_at(path_.path_index, i*d_time);
			DrawDebugPoint(GetWorld(), s.pos + FVector(0, 0, 50), 2.5, color, true);
			print_log(s.pos.ToString());
		}

		last_node = last_node->prev;
	}

	return pat;
}