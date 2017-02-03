// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "RSPaths.h"

RSPaths::RSPaths(FVector pos0_, FVector vel0_, FVector pos1_, FVector vel1_, float v_max_, float phi_max_, float L_car_) {

	pos0 = pos0_;
	pos1 = pos1_;
	vel0 = vel0_;
	vel1 = vel1_;

	v_max = v_max_;
	phi_max = phi_max_;
	L_car = L_car_;





}

RSPaths::~RSPaths()
{
}




State RSPaths::step(float delta_time) {
	t_now += delta_time;
	return state_at(t_now);
}

State RSPaths::state_at(float t) {
	return State();
}










