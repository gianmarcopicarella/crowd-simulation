// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassEntityTraitBase.h"
#include "MassEntityTypes.h"
#include "MoveToTowerTrait.generated.h"


USTRUCT()
struct FMoveToTargetFragment : public FMassFragment
{
	GENERATED_BODY()
		FVector myTarget {
		0, 0, 0
	};
};

/**
 *
 */
UCLASS()
class MYPROJECT2_API UMoveToTowerTrait : public UMassEntityTraitBase
{
	GENERATED_BODY()

protected:
	virtual void BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const override;

};
