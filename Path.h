// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

/**
 Parent class for all controllers. use step(dt) to get the next point to move to. reset starts at te beginning again. use pos(time) to get the point at time t.
 Maybe add step for velocity/acceleration if needed.
 */
class AIMULT_API Path
{
public:
	Path();


	~Path();

	virtual FVector step(float delta_time) = 0;
	virtual FVector pos(float time) = 0;
	virtual void reset();



protected:
	float t = 0;
	float time_taken;

	FVector pos0;
	FVector pos1;

	FVector vel0;
	FVector vel1;
};
