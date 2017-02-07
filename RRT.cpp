// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"

//TODO:
// * optimize path: gå igenom vägen och hoppa över onödiga noder
// * optimera alla vägar och sen hitta den kortaste ist för tvärt om?

// Sets default values
ARRT::ARRT()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	//PrimaryActorTick.bCanEverTick = true;
	//world = GetWorld();
	//playerInput = GEngine->GetFirstLocalPlayerController(world);
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

	GEngine->AddOnScreenDebugMessage(0, 500.f, FColor::Yellow, "Build RRT*.  MODEL = " + controller);

	controller_type = controller;
	int nPoints = 1000;
	int max_iters = 2 * nPoints;

	//Choose strategy
	strategy = "max speed";
	//strategy = "random speed";
	//strategy = "low speed";

	//Choose neighbourhood size
	if (controller_type == "KinematicPoint") {
		neighborhood_size = 200;			//measured in dist
	}
	else
		neighborhood_size = 3;				//measured in time


	bool optimize = false; //obs funkar ej med kinematic point och kanske inte annars heller...

	FVector start = map->start_pos;
	goal_pos = map->goal_pos;
	goal_vel = map->goal_vel;
	max_a = map->a_max;
	max_v = map->v_max;
	max_phi = map->phi_max;
	max_omega = map->omega_max;
	car_L = map->L_car;
	default_Z = map->default_Z;

	polygons = map->allGroundPoints;
	bounds = map->allWallPoints;
	generatePoints(nPoints);

	notInTree = RRTpoints;
	notInTree.Add(goal_pos);

	RRTnode* start_node = new RRTnode();
	start_node->pos = start;
	start_node->prev = NULL;
	start_node->tot_path_cost = 0;
	start_node->v = map->start_vel;
	start_node->dPath = DynamicPath();
	start_node->path = NULL;// &DynamicPath();
	inTree.Add(start_node);

	//DrawDebugPoint(GetWorld(), start + FVector(0,0,100), 10.5, FColor::Red, true);
	//DrawDebugPoint(GetWorld(), goal_pos + FVector(0, 0, 100), 10.5, FColor::Red, true);
	opt = false;

	int randIndex = 0;

	FVector tempPos1;

	RRTnode* goal = new RRTnode();
	goal->pos = FVector(NULL, NULL, NULL);
	goal->tot_path_cost = float_inf;

	bool goal_reached = false;
	int iters = 0;
	//TArray<RRTnode*> goalNodes;//array with all nodes that reached the goal
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
			//om kortare väg --> spara den
			map->print_log("goal reached. cost: " + FString::SanitizeFloat(node->tot_path_cost) + " (points left: " + FString::FromInt(notInTree.Num()) + ")");
			if (node->tot_path_cost < goal->tot_path_cost)
				goal = node;
			break; //return first goal
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
		map->print("GOAL FOUND. cost: " + FString::SanitizeFloat(goal->tot_path_cost));
		FVector trace_offset2 = FVector(0, 0, trace_offset.Z + 10.f);

		float cost = goal->tot_path_cost;
		node = goal;

		//Optimise path!
		//takes forever even if stop?
		if (optimize) {

			// ---Draw not opt. path ---
			float resolution = 100;
			while (node->prev != NULL) {
				if (controller_type == "DynamicPoint") {
					//Draw dyn path
					DynamicPath dp = node->dPath;
					float time = dp.path_time() / resolution;
					dp.reset();
					State s;
					for (int i = 0; i <= resolution; i++) {
						if (i == 0)
							s = dp.step(0);
						else
							s = dp.step(time);
						if (s.vel.Size() > map->v_max)
							map->print("over max speed! " + FString::SanitizeFloat(s.vel.Size()));
						DrawDebugPoint(GetWorld(), s.pos + trace_offset2, 2.5, FColor::Yellow, true);
					}
					//DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Blue, true, -1.f, 0, 5.f);
				}

				//DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Blue, true, -1.f, 0, 5.f);

				else if (controller_type == "KinematicPoint") {
					DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Yellow, true, -1.f, 0, 10);
				}
				node = node->prev;
			}


			// --- opt. path --- TESTA NYA V
			node = goal;
			RRTnode* node2;
			float minCost = goal->tot_path_cost;
			while (node->prev->prev != NULL) {
				node2 = node;
				while (node2->prev->prev != NULL) {
					if (controller_type == "DynamicPoint") {
						temp_dPath2.Empty();
						DynamicPath dp = calc_path(node2->prev->prev->pos, node2->prev->prev->v, node2->pos, node2->v);
						if (dp.valid && node2->prev->tot_path_cost + dp.path_time() < minCost) {
							node2->prev = node2->prev->prev;
							node2->tot_path_cost = node2->prev->tot_path_cost + dp.path_time();
							node2->cost_to_prev = dp.path_time();
							node2->dPath2 = temp_dPath2;
							node2->dPath = dp;
							minCost = node2->tot_path_cost;
						}
						else {
							FVector newV = randVel();
							DynamicPath dp = calc_path(node2->prev->prev->pos, node2->prev->prev->v, node2->pos, newV);
							if (dp.valid && node2->prev->tot_path_cost + dp.path_time() < minCost) {
								node2->prev = node2->prev->prev;
								node2->tot_path_cost = node2->prev->tot_path_cost + dp.path_time();
								node2->cost_to_prev = dp.path_time();
								node2->dPath2 = temp_dPath2;
								node2->dPath = dp;
								node2->v = newV;
								minCost = node2->tot_path_cost;
							}
						}
					}
					else if (controller_type == "KinematicPoint") {
						float d = FVector::Dist(node2->prev->prev->pos, node2->pos);
						if (node2->prev->tot_path_cost + d < minCost) {
							node2->prev = node2->prev->prev;
							node2->tot_path_cost = node2->prev->tot_path_cost + d;
							node2->cost_to_prev = d;
							minCost = node2->tot_path_cost;
						}
					}
					node2 = node2->prev;
				}
				if (node->prev->prev == NULL)
					break;
				if (node->prev->prev != NULL)
					node = node->prev;
			}
			node = goal;
			if (goal->tot_path_cost < cost)
				print("PATH OPTIMIZED. cost: " + FString::SanitizeFloat(goal->tot_path_cost));
			else
				print("PATH COULD NOT BE OPTIMIZED");
		}


		// --- Draw final path and add path to array
		TArray<RRTnode*> path;
		float resolution = 100;
		while (node->prev != NULL) {
			//print("pos: " + node->pos.ToString() + "   vel: " + node->v.ToString());
			path.Add(node);
			
			if (controller_type == "DynamicPoint") {
				//Draw dyn path
				//TArray<FVector> path_ = node->dPath2;
				DynamicPath dp = node->dPath;
				//if (p->valid)
				//	print("ok");
				//float resolution = path_.Num();// -1; ?
				float time = dp.path_time() / resolution;
				dp.reset();
				State s;
				for (int i = 0; i <= resolution; i++) {
					if (i == 0)
						s = dp.step(0);
					else
						s = dp.step(time);
					if (s.vel.Size() > map->v_max)
						map->print("over max speed! " + FString::SanitizeFloat(s.vel.Size())); //TODO: fix
					DrawDebugPoint(GetWorld(), s.pos + trace_offset2, 2.5, FColor::Red, true);
				}
			}
			else if (controller_type == "SimpleCar") {
				resolution = 1000;
				RSPaths rs = node->rsPath;
				float time = rs.path_time(rs.path_index) / resolution;
				rs.reset();
				State s;
				for (int i = 0; i <= resolution; i++) {
					time = i*rs.path_time(i) / resolution;
					s = rs.state_at(rs.path_index, time);

					//if (s.vel.Size() > map->v_max)
					//	map->print("over max speed! " + FString::SanitizeFloat(s.vel.Size())); //TODO: fix
					DrawDebugPoint(GetWorld(), s.pos + trace_offset2, 2.5, FColor::Red, true);
				}

				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Yellow, true, -1.f, 0, 5.f);
			}
			else if (controller_type == "DD") {
				resolution = 1000;
				DifferentialDrivePaths rs = node->DDpath;
				float time = rs.path_time(rs.path_index) / resolution;
				rs.reset();
				State s;
				for (int i = 0; i <= resolution; i++) {
					time = i*rs.path_time(i) / resolution;
					s = rs.state_at(rs.path_index, time);

					//if (s.vel.Size() > map->v_max)
					//	map->print("over max speed! " + FString::SanitizeFloat(s.vel.Size())); //TODO: fix
					DrawDebugPoint(GetWorld(), s.pos + trace_offset2, 2.5, FColor::Red, true);
				}

				DrawDebugLine(GetWorld(), node->pos + trace_offset2, node->prev->pos + trace_offset2, FColor::Yellow, true, -1.f, 0, 5.f);
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
	map->print("Points generated. Skipped " + FString::FromInt(numSkippedPoints) + " points of " + FString::FromInt(nPoints));
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

//float ARRT::getAngle(FVector a, FVector b) {
//	float dot = a.X*b.X + a.Y*b.Y; // dot product
//	float det = a.X*b.Y - a.Y*b.X; // determinant
//	float angle = atan2(det, dot);
//	return abs(angle);
//}

RRTnode* ARRT::findNearest(FVector pos, float max_cost) {
	// Find nearest point in tree

	TArray<RRTnode*> neighborhood;
	TArray<FVector> neighborhood_vs;
	RRTnode* newNode = new RRTnode();

	FVector v2;
	if (pos == goal_pos)
		v2 = goal_vel;

	float smallestCost = float_inf;
	float costToRootNode; //to minimize
	int nearest = -2;
	float vel = 0;
	for (int i = 0; i < inTree.Num(); i++) {

		if (pos != goal_pos) {
			v2 = randVel();
			if (v2.Size() > max_v + 0.001)
				print("v2 over max speed: " + FString::SanitizeFloat(v2.Size()));
		}

		if (controller_type == "DynamicPoint") {
			temp_dPath2.Empty();

			// generate v2 = random velocity
			DynamicPath dp = calc_path(inTree[i]->pos, inTree[i]->v, pos, v2);

			if (dp.valid) {

				if (dp.path_time() <= neighborhood_size) {
					neighborhood.Add(inTree[i]);
					neighborhood_vs.Add(v2);
				}

				costToRootNode = inTree[i]->tot_path_cost + dp.path_time();
				if (costToRootNode < smallestCost) {
					if (temp_dPath2.Num() > 0) {
						nearest = i;
						smallestCost = costToRootNode;
						newNode->path = &dp;
						newNode->dPath = dp;
						newNode->dPath2 = temp_dPath2;
						newNode->cost_to_prev = dp.path_time();
						newNode->v = v2;
					}
				}
			}
		}
		if (controller_type == "SimpleCar") {

			RSPaths rs = calc_path_RS(inTree[i]->pos, inTree[i]->v, pos, v2);

			if (rs.path_index != -1) {
				costToRootNode = inTree[i]->tot_path_cost + rs.path_time(rs.path_index);
				if (costToRootNode < smallestCost) {

					nearest = i;
					smallestCost = costToRootNode;
					newNode->rsPath = rs;
					newNode->path = &rs;
					newNode->cost_to_prev = rs.path_time(rs.path_index);
					newNode->v = v2;

				}

				if (rs.path_time(rs.path_index) <= neighborhood_size) {
					neighborhood.Add(inTree[i]);
					neighborhood_vs.Add(v2);
				}
			}
		}
		if (controller_type == "DD") {
			DifferentialDrivePaths rs = calc_path_DD(inTree[i]->pos, inTree[i]->v, pos, v2);

			if (rs.path_index != -1) {
				costToRootNode = inTree[i]->tot_path_cost + rs.path_time(rs.path_index);
				if (costToRootNode < smallestCost) {

					nearest = i;
					smallestCost = costToRootNode;
					//newNode->path = &rs;
					newNode->DDpath = rs;
					newNode->cost_to_prev = rs.path_time(rs.path_index);
					newNode->v = v2;

				}

				if (rs.path_time(rs.path_index) <= neighborhood_size) {
					neighborhood.Add(inTree[i]);
					neighborhood_vs.Add(v2);
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
				newNode->v = v2;
			}
		}
	}
	if (nearest < 0)
		return newNode;

	newNode->pos = pos;
	newNode->prev = inTree[nearest];
	newNode->tot_path_cost = smallestCost;


	//RRT* ------- check in neighbourhood if better ------
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
			//DynamicPath dp = calc_path(neighborhood[i]->pos, neighborhood[i]->v, pos, v2);
			DynamicPath dp = calc_path(neighborhood[i]->pos, neighborhood[i]->v, pos, neighborhood_vs[i]);
			if (dp.valid && temp_dPath2.Num()>0) {
				costToRootNode = neighborhood[i]->tot_path_cost + dp.path_time();
				if (costToRootNode < smallest_pathCost) {
					newNode->prev = neighborhood[i];
					newNode->tot_path_cost = costToRootNode;
					newNode->dPath2 = temp_dPath2;
					newNode->dPath = dp;
					newNode->path = &dp;
					newNode->cost_to_prev = dp.path_time();
					newNode->v = neighborhood_vs[i];
					smallest_pathCost = costToRootNode;
				}
			}
		}
		else if (controller_type == "SimpleCar") {
			RSPaths rs = calc_path_RS(neighborhood[i]->pos, neighborhood[i]->v, pos, v2);
			if (rs.path_index != -1) {
				costToRootNode = neighborhood[i]->tot_path_cost + rs.path_time(rs.path_index);
				if (costToRootNode < smallest_pathCost) {
					newNode->prev = neighborhood[i];
					newNode->tot_path_cost = costToRootNode;
					newNode->rsPath = rs;
					newNode->path = &rs;
					newNode->cost_to_prev = rs.path_time(rs.path_index);
					newNode->v = neighborhood_vs[i];
					smallest_pathCost = costToRootNode;
				}
			}
		}

	}
	//print("draw new");
	DrawDebugLine(GetWorld(), pos + trace_offset, newNode->prev->pos + trace_offset, FColor::Yellow, true, -1.f, 2.5f);
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

	//Check correct vel at start
	State s = dp.step(0);
	if (FVector::Dist(s.vel, vel0) > 0.001 && FVector::Dist(s.pos, pos0) > 0.001) {
		dp.valid = false;
		return dp;
	}

	for (int i = 0; i <= resolution; ++i) {
		s = dp.step(time);

		prevPos = s.pos;
		temp_dPath2.Add(s.pos);

		if (isInAnyPolygon(s.pos) || !isInPolygon(s.pos, boundPoints) || s.vel.Size() > max_v) {
			dp.valid = false;
			return dp;
		}
	}

	//Check correct vel at end
	if (FVector::Dist(s.vel, vel1)>0.001 && FVector::Dist(s.pos, pos1)>0.001)
		dp.valid = false;

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

FVector ARRT::randVel() {
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

	return FVector(vx, vy, 0);
}

// calculate path between two points and velocities
RSPaths ARRT::calc_path_RS(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	RSPaths rs(pos0, vel0, pos1, vel1, max_v, max_phi, car_L);

	int bestPath_index = -1;
	float resolution = 100;
	float time;
	bool valid = true;
	//Path bestPath;
	for (int i = 0; i < rs.all_paths.size(); i++) {
		State s = rs.state_at(0, i);
		valid = true;

		rs.reset();
		//check if path = valid
		for (int j = 0; j < resolution; j++) {
			if (bestPath_index != -1)
				break;
			time = j*rs.path_time(i) / resolution;
			s = rs.state_at(i, time);

			if (isInAnyPolygon(s.pos) || !isInPolygon(s.pos, boundPoints) || s.vel.Size() > max_v) {
				valid = false;
				// not valid
				break;
			}
		}
		if (valid) {//FVector::Dist(s.vel, vel1) < 0.001 && FVector::Dist(s.pos, pos1) < 0.001 && valid) {
			bestPath_index = i;
			break;
		}
	}

	rs.path_index = bestPath_index;
	return rs;
}


DifferentialDrivePaths ARRT::calc_path_DD(FVector pos0, FVector vel0, FVector pos1, FVector vel1) {
	DifferentialDrivePaths rs(pos0, vel0, pos1, vel1, max_v, max_omega);

	int bestPath_index = -1;
	float resolution = 100;
	float time;
	bool valid = true;
	//Path bestPath;
	for (int i = 0; i < rs.all_paths.size(); i++) {
		State s = rs.state_at(0, i);
		valid = true;

		rs.reset();
		//check if path = valid
		for (int j = 0; j < resolution; j++) {
			if (bestPath_index != -1)
				break;
			time = j*rs.path_time(i) / resolution;
			s = rs.state_at(i, time);

			if (isInAnyPolygon(s.pos) || !isInPolygon(s.pos, boundPoints) || s.vel.Size() > max_v) {
				valid = false;
				// not valid
				break;
			}
		}
		if (valid) {//FVector::Dist(s.vel, vel1) < 0.001 && FVector::Dist(s.pos, pos1) < 0.001 && valid) {
			bestPath_index = i;
			break;
		}
	}

	rs.path_index = bestPath_index;
	return rs;
}