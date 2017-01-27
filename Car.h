// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "GameFramework/Actor.h"
#include "Car.generated.h"

UCLASS()
class AIMULT_API ACar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ACar();

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	
	// Called every frame
	virtual void Tick( float DeltaSeconds ) override;

	
	UParticleSystemComponent *OurParticleSystem1;
	UParticleSystemComponent *OurParticleSystem2;
	UParticleSystemComponent *OurParticleSystem3;

	FVector position;
	FVector velocity;

};
