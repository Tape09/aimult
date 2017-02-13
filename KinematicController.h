// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "MyMath.h"
#include "KinematicController.generated.h"

UCLASS()
class AIMULT_API AKinematicController : public AActor {
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	AKinematicController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	void drawPath();
	void init();
	Node dijkstras();

	float t_now = 0;
	float pidx = 0;
	bool is_driving = false;

	std::vector<float> my_path_times;

	int buffer_ticks = 100;
	float time_to_init = 1.5;
	bool has_initialized = false;

	TArray<FVector> path;
	AMapGen * map;

	FVector newPos;

};
