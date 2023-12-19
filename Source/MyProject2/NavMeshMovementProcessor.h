// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


#include "CoreMinimal.h"
#include "MassEntityTraitBase.h"
#include "MassProcessor.h"
#include "NavigationPath.h"

#include "NavMeshMovementProcessor.generated.h"

UCLASS()
class MYPROJECT2_API UNavMeshMovementProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
	UNavMeshMovementProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

private:

	FMassEntityQuery myEntities;
};
