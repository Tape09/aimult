// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RSPaths.h"

RSPaths::RSPaths() {}

RSPaths::RSPaths(FVector pos0_, FVector vel0_, FVector pos1_, FVector vel1_, float v_max_, float phi_max_, float L_car_) {

	pos0 = pos0_;
	pos1 = pos1_;
	vel0 = vel0_;
	vel1 = vel1_;

	v_max = v_max_;
	phi_max = phi_max_;
	L_car = L_car_;

	turn_radius = L_car / tan(phi_max);
	//turn_radius = 2;
	//print_log(FString::SanitizeFloat(turn_radius));
	//print_log(FString::SanitizeFloat(phi_max));

	RSState goal_state((pos1 - pos0) / turn_radius, wrapAngle(vecAngle(vel1) - vecAngle(vel0)));
	rotateVector(goal_state.pos, -vecAngle(vel0));
	//goal_state.pos = goal_state.pos.RotateAngleAxis(rad2deg(-vecAngle(vel0)),FVector(0,0,1));

	//print_log(goal_state.pos.ToString());
	//print_log(FString::SanitizeFloat(goal_state.theta));

	addTransforms(std::bind(&RSPaths::get_path_LSL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LSR, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGRGL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGRL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LRGL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LRGLR, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGRLGR, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGR90SL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LSR90GL, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGR90SR, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LSL90GR, this, std::placeholders::_1), goal_state);
	addTransforms(std::bind(&RSPaths::get_path_LGR90SL90GR, this, std::placeholders::_1), goal_state);

	std::sort(all_paths.begin(), all_paths.end());
}

RSPaths::~RSPaths() {}

float RSPaths::time_taken(int idx) {
	return turn_radius * all_paths[idx].calc_dist() / v_max;
}

float RSPaths::time_taken() {
	return turn_radius * all_paths[0].calc_dist() / v_max;
}


State RSPaths::state_at(RSPath rsp, float time) const {
	std::vector<float> times;
	for (int i = 0; i<rsp.size(); ++i) {
		times.push_back(rsp.components[i].time);
	}

	for (int i = 1; i<times.size(); ++i) {
		times[i] += times[i - 1];
	}

	State istate;
	istate.pos = pos0;
	istate.vel = vel0;

	//print_log(istate);

	if (time <= times[0]) {
		return state_at(istate, rsp.components[0], time);
	}
	else {
		istate = state_at(istate, rsp.components[0], rsp.components[0].time);
	}

	//print_log(istate);

	for (int i = 1; i<times.size(); ++i) {
		if (time <= times[i]) {
			return state_at(istate, rsp.components[i], time - times[i - 1]);
		}
		else {
			istate = state_at(istate, rsp.components[i], rsp.components[i].time);
		}
		//print_log(istate);
	}

	return istate;

}

State RSPaths::state_at(State istate, RSComponent rsc, float time) const {
	State s;
	//float theta0 = vecAngle(istate.vel);


	//if (rsc.turn == S) {
	//	float dist = rsc.gear * v_max * time;

	//	s.pos.X = dist*cos(theta0) + istate.pos.X;
	//	s.pos.Y = dist*sin(theta0) + istate.pos.Y;

	//	s.vel = istate.vel;
	//	s.acc = FVector(0, 0, 0);
	//	//print_log(FString::SanitizeFloat(rsc.time));
	//	//print_log(FString::SanitizeFloat(time));

	//} else {

	//	float theta = rsc.gear * v_max * time / turn_radius;
	//	float LL = 2 * sin(theta / 2) * turn_radius;
	//	s.pos.X = LL * cos(theta / 2);
	//	s.vel.X = v_max*cos(theta);
	//	s.vel = istate.vel;

	//	if (rsc.turn == L) {
	//		s.pos.Y = LL * sin(theta / 2);
	//		rotateVector(s.vel, theta);
	//	} else {
	//		s.pos.Y = -LL * sin(theta / 2);
	//		rotateVector(s.vel, -theta);
	//	}

	//	s.vel.Normalize();
	//	s.vel = s.vel * v_max;
	//	

	//	//print_log(FString::SanitizeFloat(theta));
	//	//print_log(FString::SanitizeFloat(theta0));

	//	s.acc = FVector(0, turn_radius, 0) - s.pos;

	//	rotateVector(s.pos, theta0);
	//	rotateVector(s.acc, theta0);
	//	//rotateVector(s.vel, theta0);
	//	s.pos = s.pos + istate.pos;


	//}
	// s.pos.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));
	// s.vel.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));
	// s.acc.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));

	if (rsc.turn == L) {
		s = drive_L(istate, rsc, time);
	}
	else if (rsc.turn == R) {
		s = drive_R(istate, rsc, time);
	}
	else {
		s = drive_S(istate, rsc, time);
	}



	return s;
}

State RSPaths::drive_R(State istate, RSComponent rsc, float time) const {
	State s;
	float theta0 = vecAngle(istate.vel);

	float theta = rsc.gear * v_max * time / turn_radius;
	float LL = 2 * sin(theta / 2) * turn_radius;
	s.pos.X = LL * cos(theta / 2);
	s.pos.Y = -LL * sin(theta / 2);

	s.vel = istate.vel;
	rotateVector(s.vel, -theta);

	s.acc = FVector(0, turn_radius, 0) - s.pos;

	rotateVector(s.pos, theta0);
	rotateVector(s.acc, theta0);

	s.vel.Normalize();
	s.vel = s.vel * v_max;

	s.pos = s.pos + istate.pos;


	return s;
}

State RSPaths::drive_L(State istate, RSComponent rsc, float time) const {
	State s;
	float theta0 = vecAngle(istate.vel);

	float theta = rsc.gear * v_max * time / turn_radius;
	float LL = 2 * sin(theta / 2) * turn_radius;
	s.pos.X = LL * cos(theta / 2);
	s.pos.Y = LL * sin(theta / 2);

	s.vel = istate.vel;
	rotateVector(s.vel, theta);

	s.acc = FVector(0, turn_radius, 0) - s.pos;

	rotateVector(s.pos, theta0);
	rotateVector(s.acc, theta0);

	s.vel.Normalize();
	s.vel = s.vel * v_max;

	s.pos = s.pos + istate.pos;


	return s;
}

State RSPaths::drive_S(State istate, RSComponent rsc, float time) const {
	State s;
	float theta0 = vecAngle(istate.vel);
	float dist = rsc.gear * v_max * time;

	s.pos.X = dist*cos(theta0) + istate.pos.X;
	s.pos.Y = dist*sin(theta0) + istate.pos.Y;

	s.vel = istate.vel;
	s.acc = FVector(0, 0, 0);

	return s;
}


State RSPaths::state_at(int idx, float time) const {
	return state_at(all_paths[idx], time);
}

int RSPaths::n_paths() const {
	return all_paths.size();
}


float RSPaths::theta(float t) const {
	return v_max*t / turn_radius;
}

State RSPaths::step(float delta_time) {
	t_now += delta_time;
	return state_at(t_now);
}

State RSPaths::state_at(float t) {
	return State();
}

//1: 8.1
RSPaths::RSPath RSPaths::get_path_LSL(const RSState & goal) {
	float t = 0;
	float u = 0;
	float v = 0;

	float x = goal.pos.X - sin(goal.theta);
	float y = goal.pos.Y - 1 + cos(goal.theta);

	t = atan2(y, x);
	u = sqrt(x * x + y * y);
	v = wrapAngle(goal.theta - t);




	RSPath out_path;
	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(L, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//2: 8.2
RSPaths::RSPath RSPaths::get_path_LSR(const RSState & goal) {

	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float x = goal.pos.X + sin(goal.theta);
	float y = goal.pos.Y - 1 - cos(goal.theta);

	float u1_2 = x * x + y * y;
	float t1 = atan2(y, x);

	if (u1_2 < 4) {
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_2 - 4);
	t = wrapAngle(t1 + atan2(2, u));
	v = wrapAngle(t - goal.theta);



	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//3: 8.3
RSPaths::RSPath RSPaths::get_path_LGRGL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	//print_log(FString::SanitizeFloat(u1));
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}


	float alpha = acos(u1 / 4);
	t = mod2pi(pi / 2 + alpha + atan2(eta, xi));
	u = mod2pi(pi - 2 * alpha);
	v = mod2pi(goal.theta - t - u);

	// if (isInvalidAngle(t) || isInvalidAngle(u) || isInvalidAngle(v))
	// return float.PositiveInfinity;

	// return t + u + v;


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//4: 8.4
RSPaths::RSPath RSPaths::get_path_LGRL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}


	float alpha = acos(u1 / 4);
	t = mod2pi(pi / 2 + alpha + atan2(eta, xi));
	u = mod2pi(pi - 2 * alpha);
	v = mod2pi(t + u - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//5: 8.4
RSPaths::RSPath RSPaths::get_path_LRGL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}

	u = acos((8 - u1 * u1) / 8);
	float va = sin(u);
	float alpha = asin(2 * va / u1);
	t = mod2pi(pi / 2 - alpha + atan2(eta, xi));
	v = mod2pi(t - u - goal.theta);

	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, 1, u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);

	return out_path;
}
//6: 8.7
RSPaths::RSPath RSPaths::get_path_LRGLR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 > 4) {
		out_path.is_valid = false;
		return out_path;
	}
	float phii = atan2(eta, xi);

	if (u1 > 2) {
		float alpha = acos(u1 / 4 - 0.5);
		t = mod2pi(pi / 2 + phii - alpha);
		u = mod2pi(pi - alpha);
		v = mod2pi(goal.theta - t + 2 * u);
	}
	else {
		float alpha = acos(u1 / 4 + 0.5);
		t = mod2pi(pi / 2 + phii + alpha);
		u = mod2pi(alpha);
		v = mod2pi(goal.theta - t + 2 * u);
	}


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, 1, u));
	out_path.components.push_back(RSComponent(L, -1, -u));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);

	return out_path;
}
//7: 8.8
RSPaths::RSPath RSPaths::get_path_LGRLGR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);

	if (u1 > 6) {
		out_path.is_valid = false;
		return out_path;
	}


	float va1 = 1.25f - u1 * u1 / 16;
	if (va1 < 0 || va1 > 1) {
		out_path.is_valid = false;
		return out_path;
	}


	u = acos(va1);
	float va2 = sin(u);
	float alpha = asin(2 * va2 / u1);
	t = mod2pi(pi / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -u));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}

//8: 8.9
RSPaths::RSPath RSPaths::get_path_LGR90SL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 4) {
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_s - 4) - 2;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(2, u + 2);
	t = mod2pi(pi / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t + pi / 2 - goal.theta);



	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pi / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//9: 8.9
RSPaths::RSPath RSPaths::get_path_LSR90GL(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X - sin(goal.theta);
	float eta = goal.pos.Y - 1 + cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 4) {
		out_path.is_valid = false;
		return out_path;
	}



	u = sqrt(u1_s - 4) - 2;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(u + 2, 2);
	t = mod2pi(pi / 2 + atan2(eta, xi) - alpha);
	v = mod2pi(t - pi / 2 - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(R, 1, pi / 2));
	out_path.components.push_back(RSComponent(L, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//10: 8.10
RSPaths::RSPath RSPaths::get_path_LGR90SR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 < 2) {
		out_path.is_valid = false;
		return out_path;
	}


	t = mod2pi(pi / 2 + atan2(eta, xi));
	u = u1 - 2;
	v = mod2pi(goal.theta - t - pi / 2);

	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pi / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//11: 8.10
RSPaths::RSPath RSPaths::get_path_LSL90GR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1 = sqrt(xi * xi + eta * eta);
	if (u1 < 2) {
		out_path.is_valid = false;
		return out_path;
	}

	t = mod2pi(atan2(eta, xi));
	u = u1 - 2;
	v = mod2pi(-t - pi / 2 + goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(S, 1, u));
	out_path.components.push_back(RSComponent(L, 1, pi / 2));
	out_path.components.push_back(RSComponent(R, -1, -v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}
//12: 8.11
RSPaths::RSPath RSPaths::get_path_LGR90SL90GR(const RSState & goal) {
	RSPath out_path;

	float t = 0;
	float u = 0;
	float v = 0;

	float xi = goal.pos.X + sin(goal.theta);
	float eta = goal.pos.Y - 1 - cos(goal.theta);

	float u1_s = xi * xi + eta * eta;
	if (u1_s < 16) {
		out_path.is_valid = false;
		return out_path;
	}

	u = sqrt(u1_s - 4) - 4;
	if (u < 0) {
		out_path.is_valid = false;
		return out_path;
	}

	float alpha = atan2(2, u + 4);
	t = mod2pi(pi / 2 + atan2(eta, xi) + alpha);
	v = mod2pi(t - goal.theta);


	out_path.components.push_back(RSComponent(L, 1, t));
	out_path.components.push_back(RSComponent(R, -1, -pi / 2));
	out_path.components.push_back(RSComponent(S, -1, -u));
	out_path.components.push_back(RSComponent(L, -1, -pi / 2));
	out_path.components.push_back(RSComponent(R, 1, v));
	out_path.is_valid = true;
	out_path.calc_dist();
	out_path.calc_time(turn_radius, v_max);


	return out_path;
}



void RSPaths::addTransforms(pathFcn fptr, const RSState & goal) {
	RSPath rsp;
	RSState rs;


	rs = goal;
	rsp = fptr(rs);
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
	rs = goal;
	rs.reverse();
	rsp = fptr(rs);
	rsp.reverse();
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
	rs = goal;
	rs.reflect();
	rsp = fptr(rs);
	rsp.reflect();
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
	rs = goal;
	rs.reverse();
	rs.reflect();
	rsp = fptr(rs);
	rsp.reverse();
	rsp.reflect();
	if (rsp.is_valid) {
		all_paths.push_back(rsp);
	}
}



float RSPaths::path_time(int idx) const {
	return all_paths[idx].time;
}