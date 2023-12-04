#include "SimpleMovementFragment.h"

// Sets default values
ASimpleMovementFragment::ASimpleMovementFragment()
{
	// Set this empty to call Tick() every frame. You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ASimpleMovementFragment::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ASimpleMovementFragment::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Called to bind functionality to input
void ASimpleMovementFragment::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
}
