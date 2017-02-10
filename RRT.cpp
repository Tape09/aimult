// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RRT.h"

RRT::RRT(int max_iterations_, AMapGen * map_, PathFcnType pathFcn_, float v_max_, float a_max_) {
	map = map_;
	pathFcn = pathFcn_;
	v_max = v_max_;
	a_max = a_max_;
	max_iterations = max_iterations_;
}

RRT::~RRT()
{
}


std::vector<std::shared_ptr<RRTNode>> RRT::get_full_path() {
	std::vector<std::shared_ptr<RRTNode>> out_path;

	std::shared_ptr<RRTNode> best_path = NULL;
	float best_total_cost = 99999999;
	bool found_path = false;

	int node_idx;
	int corner_idx;
	bool visible;
	
	nodes.clear();
	nodes.push_back(std::make_shared<RRTNode>(map->start_pos, map->start_vel));

	for(int j = 0; j<max_iterations; ++j) {
		// pick random node in nodes
		node_idx = FMath::RandRange(0,nodes.size()-1);

		// linetrace corner to goal => check path to goal
		visible = map->Trace(nodes[node_idx]->corner,map->goal_pos,-1);
		if (visible) {
			std::shared_ptr<Path> temp_path = pathFcn(nodes[node_idx]->pos, nodes[node_idx]->vel, map->goal_pos, map->goal_vel);
			if (temp_path->isValid() && temp_path->pathExists()) {
				std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(nodes[node_idx],temp_path, map->goal_pos);
				if (temp_node->cost < best_total_cost) {
					best_path = temp_node;
					best_total_cost = temp_node->cost;
					found_path = true;
				}
			}
		}

	
		// line trace corner to all corners	
		std::vector<FVector> visible_corners;
		for (int i = 0; i < map->cornerPoints.Num(); ++i) {
			visible = map->Trace(nodes[node_idx]->corner,map->cornerPoints[i],-1);
			if(visible) visible_corners.push_back(map->cornerPoints[i]);
		}

		// pick corner - can pick own corner
		corner_idx = FMath::RandRange(0, visible_corners.size() - 1);

		// random point around corner
		float meanx = visible_corners[corner_idx].X;
		float meany = visible_corners[corner_idx].Y;

		std::default_random_engine generator;
		std::normal_distribution<double> distributionx(meanx, sigma2);
		std::normal_distribution<double> distributiony(meany, sigma2);
		distributionx(generator);
		distributiony(generator);
		FVector random_point = FVector(distributionx(generator), distributiony(generator),0);
		while (isInAnyPolygon(random_point, map->allGroundPoints) || !isInPolygon(random_point, map->wallPoints)) {
			random_point = FVector(distributionx(generator), distributiony(generator), 0);
		}
		FVector random_vel = randVel(v_max);

		// find points in nodes that can see corner
		std::vector<std::shared_ptr<RRTNode>> visible_nodes;
		for (int i = 0; i < nodes.size(); ++i) {
			visible = map->Trace(nodes[i]->corner, visible_corners[corner_idx], -1);
			if (visible) visible_nodes.push_back(nodes[i]);
		}

		// find paths from points to random point
		float best_cost = 999999;
		std::shared_ptr<RRTNode> best_segment;
		bool found_segment = false;
		for (int i = 0; i < visible_nodes.size(); ++i) {
			std::shared_ptr<Path> temp_path = pathFcn(visible_nodes[i]->pos, visible_nodes[i]->vel, random_point, random_vel);
			if (temp_path->isValid() && temp_path->pathExists()) {
				std::shared_ptr<RRTNode> temp_node = std::make_shared<RRTNode>(visible_nodes[i], temp_path, visible_corners[corner_idx]);
				if (temp_node->cost < best_cost) {
					best_cost = temp_node->cost;
					found_segment = true;
					best_segment = temp_node;
				}
			}
		}

		if (found_segment) {
			nodes.push_back(best_segment);
		}
	}

	if(found_path) {
		std::shared_ptr<RRTNode> asdf = best_path;
		while (asdf->cost != 0) {
			out_path.push_back(asdf);
			asdf = asdf->child;
		}

		std::reverse(out_path.begin(), out_path.end());
	}

	return out_path;
}






















