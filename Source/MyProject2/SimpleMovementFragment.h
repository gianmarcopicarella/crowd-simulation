#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "SimpleMovementFragment.generated.h"

UCLASS()
class MYPROJECT2_API ASimpleMovementFragment : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this empty's properties
	ASimpleMovementFragment();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;
};
