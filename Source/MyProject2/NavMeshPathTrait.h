// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassEntityTraitBase.h"
#include "MassEntityTypes.h"
#include "NavigationPath.h"

#include "NavMeshPathTrait.generated.h"


USTRUCT()
struct FNavMeshPathFragment : public FMassFragment
{
	GENERATED_BODY()

	int myPathIndex{ 0 };
	FVector myTargetLocation;
	FPathFindingResult myPathResult;

	bool myHasMoved{ false };

	void SetTargetLocation(const FVector& Target)
	{
		myTargetLocation = Target;
		myPathIndex = 0;
	}
};

/**
 *
 */
UCLASS()
class MYPROJECT2_API UNavMeshPathTrait : public UMassEntityTraitBase
{
	GENERATED_BODY()

protected:
	virtual void BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const override;

};