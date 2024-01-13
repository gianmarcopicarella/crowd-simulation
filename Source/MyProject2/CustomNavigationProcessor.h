// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"
#include "MassObserverProcessor.h"
#include "CustomAvoidanceProcessor.h"
#include "CustomNavigationProcessor.generated.h"

/**
 * 
 */
 /** Processor to update obstacle grid */
UCLASS()
class MYPROJECT2_API UCustomMassNavigationObstacleGridProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
	UCustomMassNavigationObstacleGridProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Initialize(UObject& Owner) override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

private:
	FMassEntityQuery AddToGridEntityQuery;
	FMassEntityQuery UpdateGridEntityQuery;
	FMassEntityQuery RemoveFromGridEntityQuery;

	TObjectPtr<UCustomMassNavigationSubsystem> CustomNavigationSubsystem;
};

/** Deinitializer processor to remove avoidance obstacles from the avoidance obstacle grid */
UCLASS()
class MYPROJECT2_API UCustomMassNavigationObstacleRemoverProcessor : public UMassObserverProcessor
{
	GENERATED_BODY()

		UCustomMassNavigationObstacleRemoverProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

	FMassEntityQuery EntityQuery;
};
