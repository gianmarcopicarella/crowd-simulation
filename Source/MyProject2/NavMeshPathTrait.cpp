// Fill out your copyright notice in the Description page of Project Settings.


#include "NavMeshPathTrait.h"
#include "MassEntityTemplateRegistry.h"

#include "MassNavigationFragments.h"

void UNavMeshPathTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	BuildContext.AddFragment<FNavMeshPathFragment>();
	BuildContext.AddFragment<FMassMoveTargetFragment>();
}