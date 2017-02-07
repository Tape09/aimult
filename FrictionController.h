// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "Car.h"
#include "FrictionCarPaths.h"
#include "MyMath.h"
#include "FrictionController.generated.h"

UCLASS()
class AIMULT_API AFrictionController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFrictionController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	void init();

	float time_to_init = 1.5;
	bool has_initialized = false;

	FrictionCarPaths calc_path(FVector pos0, FVector vel0, FVector pos1, FVector vel1);


	AMapGen * map;
	
};
