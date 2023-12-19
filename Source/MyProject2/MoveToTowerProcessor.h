// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"
#include "MassCommonTypes.h"
#include "MoveToTowerProcessor.generated.h"

/**
 * 
 */
UCLASS()
class MYPROJECT2_API UMoveToTowerProcessor : public UMassProcessor
{
	GENERATED_BODY()
	
public:
	UMoveToTowerProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

private:

	FMassEntityQuery myEntities;
};

