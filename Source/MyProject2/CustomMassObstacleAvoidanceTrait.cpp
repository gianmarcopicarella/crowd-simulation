// Fill out your copyright notice in the Description page of Project Settings.


#include "CustomMassObstacleAvoidanceTrait.h"

#include "Avoidance/MassAvoidanceFragments.h"
#include "MassEntityTemplateRegistry.h"
#include "MassMovementFragments.h"
#include "MassCommonFragments.h"
#include "MassNavigationFragments.h"
#include "Engine/World.h"
#include "MassEntityUtils.h"

void UCustomMassObstacleAvoidanceTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	FMassEntityManager& EntityManager = UE::Mass::Utils::GetEntityManagerChecked(World);

	BuildContext.RequireFragment<FAgentRadiusFragment>();
	BuildContext.AddFragment<FMassNavigationEdgesFragment>();
	BuildContext.RequireFragment<FTransformFragment>();
	BuildContext.RequireFragment<FMassVelocityFragment>();
	BuildContext.RequireFragment<FMassForceFragment>();
	BuildContext.RequireFragment<FMassMoveTargetFragment>();

	const FCustomMassMovingAvoidanceParameters MovingValidated = MovingParameters.GetValidated();
	const FConstSharedStruct MovingFragment = EntityManager.GetOrCreateConstSharedFragment(MovingValidated);
	BuildContext.AddConstSharedFragment(MovingFragment);

	const FCustomMassStandingAvoidanceParameters StandingValidated = StandingParameters.GetValidated();
	const FConstSharedStruct StandingFragment = EntityManager.GetOrCreateConstSharedFragment(StandingValidated);
	BuildContext.AddConstSharedFragment(StandingFragment);
}

void UCustomMassNavigationObstacleTrait::BuildTemplate(FMassEntityTemplateBuildContext& BuildContext, const UWorld& World) const
{
	BuildContext.RequireFragment<FAgentRadiusFragment>();

	BuildContext.AddFragment<FCustomMassNavigationObstacleGridCellLocationFragment>();
}