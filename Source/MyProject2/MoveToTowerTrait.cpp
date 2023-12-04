// Fill out your copyright notice in the Description page of Project Settings.


#include "MoveToTowerTrait.h"
#include "MassEntityTemplateRegistry.h"


void UMoveToTowerTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	BuildContext.AddFragment<FMoveToTargetFragment>();
}
