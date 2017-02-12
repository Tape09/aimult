// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "DynamicPath.h"
#include <memory>
#include "RRT.h"
#include "DynamicPointController.generated.h"

//struct DynamicPathX {
//	DynamicPathX(FVector pos0, FVector vel0, FVector pos1, FVector vel1)
//
//
//};



UCLASS()
class AIMULT_API ADynamicPointController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADynamicPointController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	//void drawPath();
	void init();



	//float time_to_init = 1.5;
	bool has_initialized = false;

	int buffer_ticks = 100;

	float v_max;
	float a_max;

	float t_now = 0;
	float pidx = 0;
	bool is_driving = false;

	std::vector<std::shared_ptr<RRTNode>> my_path;
	std::shared_ptr<Path> calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);


	std::vector<DynamicPath> paths;


	AMapGen * map;
	
};
