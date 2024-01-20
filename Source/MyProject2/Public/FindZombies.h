// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "../CustomAvoidanceProcessor.h"
#include "FindZombies.generated.h"

/**
 *
 */
UCLASS()
class MYPROJECT2_API UFindZombies : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

//		TObjectPtr<UCustomMassNavigationSubsystem> CustomNavigationSubsystem;
//
//public:
//	void Initialize(UObject& Owner) override
//	{
//		Super::Initialize(Owner);
//
//		CustomNavigationSubsystem = UWorld::GetSubsystem<UCustomMassNavigationSubsystem>(Owner.GetWorld());
//	}
//
//	UFUNCTION(BlueprintCallable)
//		static void GetZombies()
//	{
//		UE_LOG(LogTemp, Warning, TEXT("Hello World"));
//	}

};
