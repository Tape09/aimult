// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "DynamicPath.h"

DynamicPath::DynamicPath(FVector pos0, FVector vel0, FVector pos1, FVector vel1, float v_max_, float a_max_) {
	
	v_max = v_max_;
	a_max = a_max_;

	p_0 = pos0;
	p_1 = pos1;

	v_0 = vel0;
	v_1 = vel1;

	Path1D tf_x = one_dim_quadratic(p_0.X, v_0.X, p_1.X, v_1.X);
	Path1D tf_y = one_dim_quadratic(p_0.Y, v_0.Y, p_1.Y, v_1.Y);

	applyPath(tf_x, 0);
	applyPath(tf_y, 1);

	float timex = t_1[0] + t_2[0] + t_3[0];
	float timey = t_1[1] + t_2[1] + t_3[1];

	t_buffer[0] = 0;
	t_buffer[1] = 0;

	if (timex > timey) {
		tf_y = slow_path(p_0.Y, v_0.Y, p_1.Y, v_1.Y, timex);
		applyPath(tf_x, 1);

	} else if (timey > timex) {
		tf_x = slow_path(p_0.X, v_0.X, p_1.X, v_1.X, timey);
		applyPath(tf_x,0);
	}


	

	//finalx = (a_1.X * t_3x * t_3x / 2) + (a_0.X*t_1x*t_3x) + (v_0.X*t_3x) + (a_0.X * t_1x * t_2x) + (v_0.X*t_2x) + (a_0.X * t_1x * t_1x / 2) + (v_0.X * t_1x) + p_0.X;
	//v_1c = a_1 * t_3 + a_0 * t_1 + v_0;

}

DynamicPath::~DynamicPath()
{
}


FVector DynamicPath::step(float delta_time) {

	return FVector();
}

FVector DynamicPath::pos(float time) {

	return FVector();
}


FVector DynamicPath::final_pos() {
	FVector fp(0,0,p_0.Z);

	for (int i = 0; i < 2; ++i) {
		fp[i] = (a_1[i] * t_3[i] * t_3[i] / 2) + (a_0[i] *t_1[i] *t_3[i]) + (v_0[i] *t_3[i]) + (a_0[i] * t_1[i] * t_2[i]) + (v_0[i] *t_2[i]) + (a_0[i] * t_1[i] * t_1[i] / 2) + (v_0[i] * t_1[i]) + p_0[i];
	}

	return fp;
}

FVector DynamicPath::final_vel() {
	FVector fv;
	
	for (int i = 0; i < 2; ++i) {
		fv[i] = a_1[i] * t_3[i] + a_0[i] * t_1[i] + v_0[i];
	}

	return fv;
}

void DynamicPath::applyPath(Path1D p, int idx) {
	
	a_0[idx] = p.a0;
	a_1[idx] = -p.a0;
	t_1[idx] = p.t1;
	t_2[idx] = p.t2;
	t_3[idx] = p.t3;
	vm[idx] = p.vm;

}

// WORKS
DynamicPath::Path1D DynamicPath::one_dim_quadratic(float x0, float v0, float x1, float v1) {

	float a0;
	float a1;

	float vmx;

	float a;
	float b;
	float c;

	float t1 = 0;
	float t2 = 0;
	float t3 = 0;

	bool found_best = false;
	float best_time = 999999;
	float best_a0 = 99999;
	float best_t1 = 0;
	float best_t2 = 0;
	float best_t3 = 0;
	float best_vmx = 0;

	float v_mid;

	float possible_a0[] = {-a_max, a_max};
	float possible_vm[] = {-v_max,v_max};

	// ALWAYS ACCELERATE
	for (int i = 0; i < 2; ++i) {
		a0 = possible_a0[i];
		a1 = -a0;

		a = a0 / 2 - (a0*a0) / (2 * a1);
		b = v0 - (a0 / a1) * v0;
		c = (v1*v1) / (2 * a1) - (v0*v0) / (2 * a1) + x0 - x1;

		float det = b*b - 4 * a*c;
		if (det < 0) continue;

		t1 = (-b + sqrt(det)) / (2*a);
		t3 = (v1 - v0 - a0 * t1) / a1;
		
		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t3 < best_time) {
				best_time = t1 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t3 = t3;
				best_vmx = v_mid;
				found_best = true;
			}
		}

		t1 = (-b - sqrt(det)) / (2 * a);
		t3 = (v1 - v0 - a0 * t1) / a1;
		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t3 < best_time) {
				best_time = t1 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t3 = t3;
				best_vmx = v_mid;
				found_best = true;
			}
		}
	}
	


	// WITH CONSTANT VEL;
	if(!found_best) {
		for (int i = 0; i < 2; ++i) {
			a0 = possible_a0[i];
			a1 = -a0;
			vmx = possible_vm[i];

			t1 = (vmx - v0) / a0;
			t3 = (v1 - vmx) / a1;
			
			t2 = (x1-x0)/vmx - t3 - (v0/vmx)*t1 - (a1*t3*t3)/(2*vmx) - (a0*t1*t1) / (2 * vmx);

			if (t1 >= 0 && t2 >= 0 && t3 >= 0) {
				if (t1 + t2 + t3 < best_time) {
					best_time = t1 + t3;
					best_a0 = a0;
					best_t1 = t1;
					best_t2 = t2;
					best_t3 = t3;
					best_vmx = vmx;
				}
			}		
		}
	}






	return Path1D(best_t1,best_t2,best_t3, best_a0,v_max);



}


// WRONG: TODO: solve quadratic eqn, for 3 variables
DynamicPath::Path1D DynamicPath::slow_path(float x0, float v0, float x1, float v1, float time) {
	float a0;
	float a1;


	float a;
	float b;
	float c;

	float t1 = 0;
	float t2 = 0;
	float t3 = 0;

	bool found_best = false;
	float best_time = 999999;
	float best_a0 = 99999;
	float best_t1 = 0;
	float best_t2 = 0;
	float best_t3 = 0;
	float best_vmax = 0;

	float v_mid;

	float possible_a0[] = { -a_max, a_max };


	for (int i = 0; i < 2; ++i) {
		a0 = possible_a0[i];
		a1 = -a0;

		a = -a0;
		b = v1-v0 + a0*time;
		c = (v1*v1) / (2 * a1) - (v0*v0) / (2 * a1) - (v1*v0)/a1 + x0 - x1;

		float det = b*b - 4 * a*c;
		if (det < 0) continue;

		t1 = (-b + sqrt(det)) / (2 * a);
		t3 = (v1 - v0 - a0 * t1) / a1;
		t2 = time - t1 - t3;


		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}

		t1 = (-b - sqrt(det)) / (2 * a);
		t3 = (v1 - v0 - a0 * t1) / a1;
		t2 = time - t1 - t3;


		v_mid = abs(a0 * t1 + v0);

		if (t1 >= 0 && t2 >= 0 && t3 >= 0 && v_mid <= v_max) {
			if (t1 + t2 + t3 < best_time) {
				best_time = t1 + t2 + t3;
				best_a0 = a0;
				best_t1 = t1;
				best_t2 = t2;
				best_t3 = t3;
				best_vmax = v_mid;
				found_best = true;
			}
		}
	}

	return Path1D(best_t1, best_t2, best_t3, best_a0, best_vmax);
}


