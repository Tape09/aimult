// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "Path.h"
#include <vector>
#include "MyMath.h"
#include <functional>
#include <algorithm>





class AIMULT_API DifferentialDrivePaths : public Path {
public:

	enum Turn { S, L, R };

	struct RSComponent {
		RSComponent(Turn turn_, int gear_, float angle_, float w_max_, float v_, float r_) : turn(turn_), gear(gear_), angle(angle_), w_max(w_max_), v(v_), r(r_) {
			if (angle*gear < 0) {
				dist = pii + pii - abs(angle);
			} else {
				dist = abs(angle);
			}

			//r = v/w_max;

			time = dist*r/v;
		}

		Turn turn;
		int gear;
		float angle;
		float w_max;
		float v;
		float r;


		float time;
		float dist;

		void reverse() { gear = -gear; }

		void reflect() {
			if (turn == L) {
				turn = R;
			} else if (turn == R) {
				turn = L;
			}
		}

		FString toString() {
			FString trn;
			if (turn == L) {
				trn = "L";
			} else if (turn == S) {
				trn = "S";
			} else {
				trn = "R";
			}

			FString out = trn + "::" + FString::FromInt(gear) + "::" + FString::SanitizeFloat(dist) + "::" + FString::SanitizeFloat(angle);
			return out;
		}
	};

	struct RSState {
		RSState(FVector pos_ = FVector(0, 0, 0), float theta_ = 0, int gear_ = 1) : pos(pos_), theta(theta_), gear(gear_) {}

		FVector pos;
		float theta;
		//float phi;
		int gear;

		void reverse() {
			pos.X = -pos.X;
			theta = -theta;
		}

		void reflect() {
			pos.Y = -pos.Y;
			theta = -theta;
		}
	};

	struct RSPath {
		std::vector<RSComponent> components;
		bool is_valid;
		float dist;
		float time;

		float calc_time() {
			time = 0;
			for (int i = 0; i < components.size(); ++i) {
				time += components[i].time;
			}
			return time;
		}

		float calc_dist() {
			float d = 0;
			for (int i = 0; i < components.size(); ++i) {
				d += components[i].dist;
			}
			dist = d;
			return d;
		}

		void reverse() {
			for (int i = 0; i < components.size(); ++i) {
				components[i].reverse();
			}
		}

		void reflect() {
			for (int i = 0; i < components.size(); ++i) {
				components[i].reflect();
			}
		}

		bool operator<(const RSPath & other) const {
			return time<other.time;
		}

		int size() { return components.size(); }

		FString word() {
			FString out = "";
			out += FString::SanitizeFloat(components[0].r);
			out += ": ";
			for (int i = 0; i < components.size(); ++i) {
				if (components[i].turn == L) {
					out += "L";
				} else if (components[i].turn == S) {
					out += "S";
				} else {
					out += "R";
				}

				if (components[i].gear == 1) {
					out += "f";
				} else {
					out += "b";
				}



			}
			return out;
		}

	};

	//float theta(float t) const;

	State drive_R(State istate, RSComponent rsc, float time) const;
	State drive_L(State istate, RSComponent rsc, float time) const;
	State drive_S(State istate, RSComponent rsc, float time) const;

	float path_time(int idx) const;
	virtual float path_time() const;
	State state_at(State istate, RSComponent rsc, float time) const;
	State state_at(RSPath rsp, float time) const;
	State state_at(int idx, float time) const;
	int n_paths() const;

	typedef std::function<RSPath(const RSState & goal)> pathFcn;

	//1: 8.1
	RSPath get_path_LSL(const RSState & goal);
	//2: 8.2
	RSPath get_path_LSR(const RSState & goal);
	//3: 8.3
	RSPath get_path_LGRGL(const RSState & goal);
	//4: 8.4
	RSPath get_path_LGRL(const RSState & goal);
	//5: 8.4
	RSPath get_path_LRGL(const RSState & goal);
	//6: 8.7
	RSPath get_path_LRGLR(const RSState & goal);
	//7: 8.8
	RSPath get_path_LGRLGR(const RSState & goal);
	//8: 8.9
	RSPath get_path_LGR90SL(const RSState & goal);
	//9: 8.9
	RSPath get_path_LSR90GL(const RSState & goal);
	//10: 8.10
	RSPath get_path_LGR90SR(const RSState & goal);
	//11: 8.10
	RSPath get_path_LSL90GR(const RSState & goal);
	//12: 8.11
	RSPath get_path_LGR90SL90GR(const RSState & goal);

	void addTransforms(pathFcn fptr, const RSState & goal);

	// void fixVector(FVector & fv);
	// void fixState(State & s);


	DifferentialDrivePaths(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float w_max);
	~DifferentialDrivePaths();

	virtual State step(float delta_time);
	virtual State state_at(float time);

	std::vector<RSPath> all_paths;
	float time_taken();
	float time_taken(int idx);

	//FVector pos0;
	//FVector pos1;

	//FVector vel0;
	//FVector vel1;

	float r;
	float v_now;
	float v_max;
	float w_max;

	//float turn_radius;

	int path_index;
};
