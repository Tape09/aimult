// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "Path.h"
#include <vector>







class AIMULT_API RSPaths : public Path
{
public:

	enum Turn {S,L,R};

	struct RSComponent {
		RSComponent(Turn turn_, int gear_, float dist_) : turn(turn_), gear(gear_), dist(dist_) {}
		
		Turn turn;
		int gear;
		float dist;

		
	};

	struct RSState {
		FVector pos;
		FVector orient;
		float phi;
	};

	struct RSPath {
		std::vector<RSComponent> components;
		float dist;

		float calc_dist() {
			float d = 0;
			for (int i = 0; i < components.size(); ++i) {
				d += components[i].dist;
			}
			dist = d;
			return d;
		}

		bool operator<(const RSPath & other) const {
			return dist<other.dist;
		}
	};

	RSPaths(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float phi_max, float L_car);
	~RSPaths();

	virtual State step(float delta_time);
	virtual State state_at(float time);



	FVector pos0;
	FVector pos1;

	FVector vel0;
	FVector vel1;

	float v_max;
	float phi_max;
	float L_car;

};
