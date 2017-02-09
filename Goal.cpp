// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "Goal.h"


// Sets default values
AGoal::AGoal()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	USphereComponent* SphereComponent = CreateDefaultSubobject<USphereComponent>(TEXT("RootComponent"));
	RootComponent = SphereComponent;
	SphereComponent->InitSphereRadius(10.0f);
	SphereComponent->SetCollisionProfileName(TEXT("Pawn"));

	UStaticMeshComponent* SphereVisual = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualRepresentation"));
	SphereVisual->SetupAttachment(RootComponent);
	//static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Props/SM_Bush.SM_Bush"));
	//static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Props/SM_PillarFrame300.SM_PillarFrame300"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereVisualAsset(TEXT("/Game/StarterContent/Shapes/Shape_QuadPyramid.Shape_QuadPyramid"));
	
	if (SphereVisualAsset.Succeeded()) {
		SphereVisual->SetStaticMesh(SphereVisualAsset.Object);
		SphereVisual->SetRelativeRotation(FRotator(0, 180, 0));
		SphereVisual->SetRelativeLocation(FVector(0.0f, 0.0f, -10.0f));
		//SphereVisual->SetWorldScale3D(FVector(0.5f));
		SphereVisual->SetWorldScale3D(FVector(1.0f));
		SphereVisual->CastShadow = false; //no shadows!
	}

	// Create a particle system that we can activate or deactivate
	OurParticleSystem1 = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("MovementParticles"));
	OurParticleSystem1->SetupAttachment(SphereVisual);
	OurParticleSystem1->bAutoActivate = true;
	OurParticleSystem1->SetRelativeLocation(FVector(-0.0f, 0.0f, 100.0f));
	static ConstructorHelpers::FObjectFinder<UParticleSystem> ParticleAsset(TEXT("/Game/StarterContent/Particles/P_Fire.P_Fire"));
	if (ParticleAsset.Succeeded()) {
		OurParticleSystem1->SetTemplate(ParticleAsset.Object);
	}

	//static ConstructorHelpers::FObjectFinder<UMaterial> Material(TEXT("/Game/StarterContent/Materials/M_Brick_Clay_Beveled.M_Brick_Clay_Beveled"));
	static ConstructorHelpers::FObjectFinder<UMaterial> Material(TEXT("/Game/StarterContent/Materials/M_Basic_Floor.M_Basic_Floor"));
	UMaterial* mat;
	if (Material.Object != NULL)
	{
		mat = (UMaterial*)Material.Object;
		SphereVisual->SetMaterial(0,mat);
	}
}

// Called when the game starts or when spawned
void AGoal::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AGoal::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

