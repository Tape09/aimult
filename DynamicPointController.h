// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "DynamicPath.h"
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



	float time_to_init = 1.5;
	bool has_initialized = false;
	//bool move_car(FVector pos0, FVector vel0, FVector acc0, FVector pos1, FVector vel1, FVector acc1);
	bool calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);



	TArray<FVector> path;
	AMapGen * map;
	
};
