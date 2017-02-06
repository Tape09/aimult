//// Fill out your copyright notice in the Description page of Project Settings.
//
#include "aimult.h"
#include "DifferentialDrivePaths.h"
//
//DifferentialDrivePaths::DifferentialDrivePaths(FVector pos0_, FVector vel0_, FVector pos1_, FVector vel1_, float v_max_, float w_max_) {
//
//	pos0 = pos0_;
//	pos1 = pos1_;
//	vel0 = vel0_;
//	vel1 = vel1_;
//
//	v_max = v_max_;
//	w_max = w_max_;
//	L_car = 0;	
//
//	turn_radius = 1;
//
//	RSState goal_state((pos1 - pos0)/turn_radius, wrapAngle(vecAngle(vel1) - vecAngle(vel0)));
//	goal_state.pos.RotateAngleAxis(rad2deg(-vecAngle(vel0)),FVector(0,0,1));
//	
//
//	addTransforms(get_path_LSL,goal_state);
//	addTransforms(get_path_LSR,goal_state);
//	addTransforms(get_path_LGRGL,goal_state);
//	addTransforms(get_path_LGRL,goal_state);
//	addTransforms(get_path_LRGL,goal_state);
//	addTransforms(get_path_LRGLR,goal_state);
//	addTransforms(get_path_LGRLGR,goal_state);
//	addTransforms(get_path_LGR90SL,goal_state);
//	addTransforms(get_path_LSR90GL,goal_state);
//	addTransforms(get_path_LGR90SR,goal_state);
//	addTransforms(get_path_LSL90GR,goal_state);
//	addTransforms(get_path_LGR90SL90GR,goal_state);
//	
//	// sort all_paths desc
//}
//
//DifferentialDrivePaths::~DifferentialDrivePaths()
//{
//}
//
//
//State state_at(RSPath rsp, float time) const  {
//	vector<float> times;
//	for(int i = 0; i<rsp.size(); ++i) {
//		times.push_back(rsp.components[i].dist * turn_radius / v_max);
//	}
//	
//	for(int i = 1; i<times.size(); ++i) {
//		times[i] += times[i-1];
//	}
//	
//	if(time <= times[0]) return state_at(rsp.components[i],time);
//	for(int i = 1; i<times.size(); ++i) {
//		if(time <= times[i]) return state_at(rsp.components[i],time-times[i-1]);
//	}
//	
//	return State();
//	
//}
//
//State DifferentialDrivePaths::state_at(RSComponent rsc, float time) {
//	State s;
//	
//	float theta = rsc.gear * v_max * time / turn_radius;
//	float L = 2 * sin(theta/2) * turn_radius;
//	s.pos.X = L * cos(theta/2);
//	s.pos.Y = L * cos(sin/2);
//	
//	s.vel.X = cos(theta);
//	s.vel.Y = sin(theta);
//	
//	s.acc = FVector(0,1,0) - s.pos;
//	
//	s.pos.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));
//	s.vel.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));
//	s.acc.RotateAngleAxis(rad2deg(vecAngle(vel0)),FVector(0,0,1));
//	
//	return s;	
//}
//
//State DifferentialDrivePaths::state_at(int idx, float time) const {
//	return state_at(all_paths[i],time);
//}
//
//int DifferentialDrivePaths::n_paths() const {
//	return all_paths.size();
//}
//
//
//float DifferentialDrivePaths::theta(float t) const {
//	return v_max*t / turn_radius;
//}
//
//State DifferentialDrivePaths::step(float delta_time) {
//	t_now += delta_time;
//	return state_at(t_now);
//}
//
//State DifferentialDrivePaths::state_at(float t) {
//	return State();
//}
//
////1: 8.1
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LSL(const RSState & goal) {
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float x = goal.pos.X - sin(goal.theta);
//	float y = goal.pos.Y - 1 + cos(goal.theta);
//	
//	t = atan2(y, x);
//	u = sqrt(x * x + y * y);
//	v = wrapAngle(goal.theta - t);
//
//
//
//
//	RSPath out_path;
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(S, 1, u));
//	out_path.components.push_back(RSComponent(L, 1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////2: 8.2
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LSR(const RSState & goal) {
//	
//	RSPath out_path;
//	
//	float t = 0;
//	float u = 0;
//	float v = 0;
//
//	float x = goal.pos.X - sin(goal.theta);
//	float y = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1_2 = x * x + y * y;
//	float t1 = atan2(y, x);
//
//	if (u1_2 < 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//	u = sqrt(u1_2 - 4);
//	// float phi = (float)Math.Atan2(2, u);
//	t = wrapAngle(t1 + tan2(2, u));
//	v = wrapAngle(t - goal.theta);
//
//	// if (isInvalidAngle(t) || isInvalidAngle(v))
//		// return float.PositiveInfinity;
//
//	// return t + u + v;
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(S, 1, u));
//	out_path.components.push_back(RSComponent(R, 1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////3: 8.3
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGRGL(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X - sin(goal.theta);
//	float eta = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 > 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//
//	float alpha = acos(u1 / 4);
//	t = mod2pi(pi/2 + alpha + atan2(eta, xi));
//	u = mod2pi(pi - 2 * alpha);
//	v = mod2pi(goal.theta - t - u);
//
//	// if (isInvalidAngle(t) || isInvalidAngle(u) || isInvalidAngle(v))
//		// return float.PositiveInfinity;
//
//	// return t + u + v;
//	
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, u));
//	out_path.components.push_back(RSComponent(L, 1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////4: 8.4
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGRL(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X - sin(goal.theta);
//	float eta = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 > 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//	
//	float alpha = acos(u1 / 4);
//	t = mod2pi(pi/2 + alpha + atan2(eta, xi));
//	u = mod2pi(pi - 2 * alpha);
//	v = mod2pi(t + u - goal.theta);
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, u));
//	out_path.components.push_back(RSComponent(L, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////5: 8.4
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LRGL(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X - sin(goal.theta);
//	float eta = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 > 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//		
//	u = acos((8 - u1 * u1) / 8);
//	float va = sin(u);
//	float alpha = asin(2 * va / u1);
//	t = mod2pi(pi/2 - alpha + atan2(eta, xi));
//	v = mod2pi(t - u - goal.theta);
//
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, 1, u));
//	out_path.components.push_back(RSComponent(L, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//	return out_path;
//}
////6: 8.7
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LRGLR(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X + sin(goal.theta);
//	float eta = goal.pos.Y - 1 - cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 > 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	float phii = atan2(eta, xi);
//
//	if (u1 > 2)
//	{
//		float alpha = acos(u1 / 4 - 0.5);
//		t = mod2pi(pi/2 + phi - alpha);
//		u = mod2pi(pi - alpha);
//		v = mod2pi(goal.theta - t + 2 * u);
//	}
//	else
//	{
//		float alpha = acos(u1 / 4 + 0.5);
//		t = mod2pi(pi/2 + phi + alpha);
//		u = mod2pi(alpha);
//		v = mod2pi(goal.theta - t + 2 * u);
//	}
//	
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, 1, u));
//	out_path.components.push_back(RSComponent(L, -1, u));
//	out_path.components.push_back(RSComponent(R, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////7: 8.8
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGRLGR(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X + sin(goal.theta);
//	float eta = goal.pos.Y - 1 - cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	
//	if (u1 > 6) {
//		out_path.isValid = false;
//		return out_path;
//	}
//
//	
//	float va1 = 1.25f - u1 * u1 / 16;
//	if (va1 < 0 || va1 > 1) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//
//	u = acos(va1);
//	float va2 = sin(u);
//	float alpha = asin(2 * va2 / u1);
//	t = mod2pi(pi/2 + atan2(eta, xi) + alpha);
//	v = mod2pi(t - goal.theta);
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, u));
//	out_path.components.push_back(RSComponent(L, -1, u));
//	out_path.components.push_back(RSComponent(R, 1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
//
////8: 8.9
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGR90SL(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X - sin(goal.theta);
//	float eta = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1_s = xi * xi + eta * eta;
//	if (u1_s < 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//	u = sqrt(u1_s - 4) - 2;
//	if (u < 0) {
//		out_path.isValid = false;
//		return out_path;	
//	}
//		
//	float alpha = atan2(2, u + 2);
//	t = mod2pi(pi/2 + atan2(eta, xi) + alpha);
//	v = mod2pi(t + pi/2 - goal.theta);
//
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, pi/2));
//	out_path.components.push_back(RSComponent(S, -1, u));
//	out_path.components.push_back(RSComponent(L, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////9: 8.9
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LSR90GL(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X - sin(goal.theta);
//	float eta = goal.pos.Y - 1 + cos(goal.theta);
//
//	float u1_s = xi * xi + eta * eta;
//	if (u1_s < 4) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//
//
//	u = sqrt(u1_s - 4) - 2;
//	if (u < 0) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//	float alpha = atan2(u + 2, 2);
//	t = mod2pi(pi/2 + atan2(eta, xi) - alpha);
//	v = mod2pi(t - pi/2 - goal.theta);
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(S, 1, u));
//	out_path.components.push_back(RSComponent(R, 1, pi/2));
//	out_path.components.push_back(RSComponent(L, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////10: 8.10
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGR90SR(const RSState & goal) {
//	RSPath out_path;
//
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X + sin(goal.theta);
//	float eta = goal.pos.Y - 1 - cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 < 2) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//
//	t = mod2pi(pi/2 + atan2(eta, xi));
//	u = u1 - 2;
//	v = mod2pi(goal.theta - t - pi/2);
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, pi/2));
//	out_path.components.push_back(RSComponent(S, -1, u));
//	out_path.components.push_back(RSComponent(R, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////11: 8.10
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LSL90GR(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X + sin(goal.theta);
//	float eta = goal.pos.Y - 1 - cos(goal.theta);
//
//	float u1 = sqrt(xi * xi + eta * eta);
//	if (u1 < 2) {
//		out_path.isValid = false;
//		return out_path;
//	}
//		
//	t = mod2pi(atan2(eta, xi));
//	u = u1 - 2;
//	v = mod2pi(-t - pi/2 + goal.theta);
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(S, 1, u));
//	out_path.components.push_back(RSComponent(L, 1, pi/2));
//	out_path.components.push_back(RSComponent(R, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
////12: 8.11
//DifferentialDrivePaths::RSPath DifferentialDrivePaths::get_path_LGR90SL90GR(const RSState & goal) {
//	RSPath out_path;
//	
//	float t = 0; 
//	float u = 0; 
//	float v = 0;
//
//	float xi = goal.pos.X + sin(goal.theta);
//	float eta = goal.pos.Y - 1 - cos(goal.theta);
//
//	float u1_s = xi * xi + eta * eta;
//	if (u1_s < 16) {
//		out_path.isValid = false;
//		return out_path;
//	}
//
//	u = sqrt(u1squared - 4) - 4;
//	if (u < 0) {
//		out_path.isValid = false;
//		return out_path;
//	}
//	
//	float alpha = atan2(2, u + 4);
//	t = mod2pi(pi/2 + atan2(eta, xi) + alpha);
//	v = mod2pi(t - goal.theta);
//
//	
//	out_path.components.push_back(RSComponent(L, 1, t));
//	out_path.components.push_back(RSComponent(R, -1, pi/2));
//	out_path.components.push_back(RSComponent(S, -1, u));
//	out_path.components.push_back(RSComponent(L, -1, pi/2));
//	out_path.components.push_back(RSComponent(R, -1, v));
//	out_path.isValid = true;
//	out_path.calc_dist();
//
//
//	return out_path;
//}
//
//
//
//void DifferentialDrivePaths::addTransforms(void * fptr, const RSState & goal) {
//	RSPath rsp;
//	RSState rs;
//	
//	
//	rs = goal;
//	rsp = fptr->(rs);
//	if(rsp.isValid()) {
//		all_paths.push_back(rsp);
//	}
//	rs = goal;
//	rs.reverse();
//	rsp = fptr->(rs);
//	if(rsp.isValid()) {
//		all_paths.push_back(rsp);
//	}
//	rs = goal;
//	rs.reflect();
//	rsp = fptr->(rs);
//	if(rsp.isValid()) {
//		all_paths.push_back(rsp);
//	}
//	rs = goal;
//	rs.reverse();
//	rs.reflect();
//	rsp = fptr->(rs);
//	if(rsp.isValid()) {
//		all_paths.push_back(rsp);
//	}
//}
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
