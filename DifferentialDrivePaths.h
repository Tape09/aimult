//// Fill out your copyright notice in the Description page of Project Settings.
//
#pragma once
//
///**
// * 
// */
//class AIMULT_API DifferentialDrivePaths
//{
//public:
//	enum Turn {S,L,R};
//
//	struct RSComponent {
//		RSComponent(Turn turn_, int gear_, float dist_, float vel_, float w_max_) : turn(turn_), gear(gear_), dist(dist_), vel(vel_), w_max(w_max_) {}
//		
//		Turn turn;
//		int gear;
//		float dist;
//		float vel;
//		float w_max;
//		
//		float time;		
//		float turn_radius;
//		
//		
//		void reverse() {gear = -gear;}
//
//		void reflect() {
//			if (turn == L) {
//				turn = R;
//			} else if(turn == R) {
//				turn = L;
//			}
//		}
//
//		FString toString() {
//			FString trn;
//			if (turn == L) {
//				trn = "L";
//			} else if (turn == S) {
//				trn = "S";
//			} else {
//				trn = "R";
//			}
//
//			FString out = trn + "::" + FString::FromInt(gear) + "::" + FString::SanitizeFloat(dist);
//			return out;
//		}
//	};
//
//	struct RSState {
//		RSState(FVector pos_,float theta_, float phi_= 0, int gear_= 1) : pos(pos_), theta(theta_), phi(phi_), gear(gear_) {}
//
//		FVector pos;
//		float theta;
//		float phi;
//		int gear;
//		
//		void reverse() {
//			pos.X = -pos.X;
//			theta = -theta;
//		}
//		
//		void reflect() {
//			pos.Y = -pos.Y;
//			theta = -theta;
//		}
//	};
//
//	struct RSPath {
//		std::vector<RSComponent> components;
//		bool isValid;
//		float dist;
//
//		float calc_dist() {
//			float d = 0;
//			for (int i = 0; i < components.size(); ++i) {
//				d += components[i].dist;
//			}
//			dist = d;
//			return d;
//		}
//
//		void reverse() {
//			for (int i = 0; i < components.size(); ++i) {
//				components[i].reverse();
//			}
//		}
//
//		void reflect() {
//			for (int i = 0; i < components.size(); ++i) {
//				components[i].reflect();
//			}
//		}
//
//		bool operator<(const RSPath & other) const {
//			return dist<other.dist;
//		}
//
//		int size() {return components.size(); }
//
//	};
//
//	float theta(float t) const;
//	
//	
//		
//	State state_at(RSComponent rsc, float time) const ;
//	State state_at(RSPath rsp, float time) const ;
//	State state_at(int idx, float time) const ;
//	int n_paths() const;
//	
//	
//	//1: 8.1
//	RSPath get_path_LSL(const RSState & goal);
//	//2: 8.2
//	RSPath get_path_LSR(const RSState & goal);
//	//3: 8.3
//	RSPath get_path_LGRGL(const RSState & goal);
//	//4: 8.4
//	RSPath get_path_LGRL(const RSState & goal);
//	//5: 8.4
//	RSPath get_path_LRGL(const RSState & goal);
//	//6: 8.7
//	RSPath get_path_LRGLR(const RSState & goal);
//	//7: 8.8
//	RSPath get_path_LGRLGR(const RSState & goal);
//	//8: 8.9
//	RSPath get_path_LGR90SL(const RSState & goal);
//	//9: 8.9
//	RSPath get_path_LSR90GL(const RSState & goal);
//	//10: 8.10
//	RSPath get_path_LGR90SR(const RSState & goal);
//	//11: 8.10
//	RSPath get_path_LSL90GR(const RSState & goal);
//	//12: 8.11
//	RSPath get_path_LGR90SL90GR(const RSState & goal);
//
//	void addTransforms(void * fptr, const RSState & goal);
//
//	DifferentialDrivePaths(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float omega_max);
//	~DifferentialDrivePaths();
//
//	virtual State step(float delta_time);
//	virtual State state_at(float time);
//
//	std::vector<RSPath> all_paths;
//
//	FVector pos0;
//	FVector pos1;
//
//	FVector vel0;
//	FVector vel1;
//
//	float v_max;
//	float w_max;
//	float L_car;
//	float turn_radius;
//
//	int path_index;
//};
