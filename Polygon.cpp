// Fill out your copyright notice in the Description page of Project Settings.

#include "aimult.h"
#include "Polygon.h"



// Sets default values
APolygon::APolygon()
{
	//GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Cyan, "new polygon");
	mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("GeneratedMesh"));
	RootComponent = mesh;

}

// Called when the game starts or when spawned
void APolygon::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void APolygon::Tick( float DeltaTime )
{
	Super::Tick( DeltaTime );

}

void APolygon::init(TArray<FVector> groundVertices, FVector newPosition) {
	TArray<FVector> vertices;
	TArray<FColor> vertexColors;

	for (int i = 0; i < groundVertices.Num(); ++i) {
		FVector temp = groundVertices[i];
		temp.Z = 0;

		vertices.Add(groundVertices[i]);
		vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));

		
		temp.Z = 250;
		vertices.Add(temp);
		vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0)); // does nothing?
	}

	TArray<int32> Triangles;

	// floor
	for (int i = 1; i < vertices.Num()/2 - 1; ++i) {
		Triangles.Add(0);
		Triangles.Add(i*2);
		Triangles.Add(i*2 + 2);

		Triangles.Add(i*2 + 2);
		Triangles.Add(i*2);
		Triangles.Add(0);		
	}

	// ceiling
	for (int i = 1; i < vertices.Num()/2 - 1; ++i) {
		Triangles.Add(1);
		Triangles.Add(i*2+1);
		Triangles.Add(i*2 + 1 + 2);

		Triangles.Add(i*2 + 1 + 2);
		Triangles.Add(i*2 + 1);
		Triangles.Add(1);
	}

	int n = vertices.Num();
	// walls
	for (int i = 0; i < n - 2; ++i) {
		Triangles.Add(i);
		Triangles.Add(i + 1);
		Triangles.Add(i + 2);

		Triangles.Add(i + 2);
		Triangles.Add(i + 1);
		Triangles.Add(i);
	}

	Triangles.Add(n - 2);
	Triangles.Add(n - 1);
	Triangles.Add(0);

	Triangles.Add(0);
	Triangles.Add(n - 1);
	Triangles.Add(n - 2);

	Triangles.Add(n - 1);
	Triangles.Add(0);
	Triangles.Add(1);

	Triangles.Add(1);
	Triangles.Add(0);
	Triangles.Add(n - 1);

	


	TArray<FVector> normals;
	normals.AddUninitialized(vertices.Num());

	TArray<FVector2D> UV0;
	UV0.AddUninitialized(vertices.Num());

	TArray<FProcMeshTangent> tangents;
	tangents.AddUninitialized(vertices.Num());


	//TArray<FVector> vertices;

	//vertices.Add(FVector(0, 0, 0));
	//vertices.Add(FVector(100, 0, 0));
	//vertices.Add(FVector(0, 0, 100));
	//vertices.Add(FVector(100, 0, 100));
	////vertices.Add(FVector(100, 100, 0));
	////vertices.Add(FVector(0, 0, 100));
	////vertices.Add(FVector(0, 100, 100));
	////vertices.Add(FVector(100, 0, 100));
	////vertices.Add(FVector(100, 100, 100));

	//TArray<int32> Triangles;
	//Triangles.Add(0);
	//Triangles.Add(1);
	//Triangles.Add(2);
	//Triangles.Add(2);
	//Triangles.Add(1);
	//Triangles.Add(0);

	//Triangles.Add(1);
	//Triangles.Add(2);
	//Triangles.Add(3);
	//Triangles.Add(3);
	//Triangles.Add(2);
	//Triangles.Add(1);



	//TArray<FVector> normals;
	//normals.AddUninitialized(4);
	////normals.Add(FVector(1, 0, 0));
	////normals.Add(FVector(1, 0, 0));
	////normals.Add(FVector(1, 0, 0));

	//TArray<FVector2D> UV0;
	//UV0.AddUninitialized(4);
	////UV0.Add(FVector2D(0, 0));
	////UV0.Add(FVector2D(10, 0));
	////UV0.Add(FVector2D(0, 10));

	//TArray<FColor> vertexColors;
	//vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));
	//vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));
	//vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));
	//vertexColors.Add(FColor(255.0, 0.00, 0.0, 1.0));
	////vertexColors.Add(FColor(0.75, 0.75, 0.75, 1.0));
	////vertexColors.Add(FColor(0.75, 0.75, 0.75, 1.0));

	//TArray<FProcMeshTangent> tangents;
	//tangents.AddUninitialized(4);
	////tangents.Add(FProcMeshTangent(0, 1, 0));
	////tangents.Add(FProcMeshTangent(0, 1, 0));
	////tangents.Add(FProcMeshTangent(0, 1, 0));




	mesh->CreateMeshSection(1, vertices, Triangles, normals, UV0, vertexColors, tangents, true);
	SetActorLocation(newPosition, false);
}