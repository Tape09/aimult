// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "Path.h"
#include <utility>
#include <cmath>

/**
 * 
 */
class AIMULT_API DynamicPath : public Path
{
public:
	DynamicPath(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max, float a_max);
	~DynamicPath();

	virtual FVector step(float delta_time);
	virtual FVector pos(float time);


	
	struct Path1D {
		Path1D(float t1_ = 0, float t2_ = 0, float t3_ = 0,float a0_ = 0, float vm_ = 0) : t1(t1_), t2(t2_), t3(t3_), a0(a0_), vm(vm_) {}

		float t1;
		float t2;
		float t3;
		float a0;
		float vm;
	};


	Path1D one_dim_quadratic(float x0, float v0, float x1, float v1);
	Path1D slow_path(float x0, float v0, float x1, float v1, float time);

	void applyPath(Path1D p, int idx);
	FVector final_pos();
	FVector final_vel();

	FVector v_0;
	FVector v_1;

	FVector p_0;
	FVector p_1;

	float t_buffer[2];
	float t_1[2];
	float t_2[2];
	float t_3[2];
	float vm[2];

	float t_1x;
	float t_2x;
	float t_3x;

	float t_1y;
	float t_2y;
	float t_3y;

	float v_max;
	float a_max;

	float vm_x;
	float vm_y;

	FVector a_0;
	FVector a_1;

	float finalx;
	float finaly;
	float finalvx;
	float finalvy;


private:


};
