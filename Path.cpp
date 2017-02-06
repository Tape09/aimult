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

void print_log(State s) {
	print_log("pos: " + s.pos.ToString());
	print_log("vel: " + s.vel.ToString());
	print_log("acc: " + s.acc.ToString());
}