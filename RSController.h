// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "MapGen.h"
#include "RSPaths.h"
#include "RSRRT.h"
#include "RSController.generated.h"

UCLASS()
class AIMULT_API ARSController : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ARSController();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Called every frame
	virtual void Tick(float DeltaSeconds) override;

	void init();

	float time_to_init = 1.5;
	bool has_initialized = false;

	AMapGen * map;
	ARSRRT* RRT;

};