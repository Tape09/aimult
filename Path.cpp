// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "Path.h"

Path::Path()
{
}

Path::~Path()
{
}

void Path::reset() {
	t_now = 0;
}

float Path::path_time() const {
	return time_taken;
}

bool Path::is_valid() const {
	return valid;
}

bool Path::path_exists() const {
	return exists;
}

FVector Path::final_pos() const {
	return pos1;
}

FVector Path::final_vel() const {
	return vel1;
}