// Fill out your copyright notice in the Description page of Project Settings.


#include "MoveToTowerProcessor.h"
#include "MoveToTowerTrait.h"
#include "MassCommonFragments.h"
#include "MassExecutionContext.h"
#include "MassEntityManager.h"


UMoveToTowerProcessor::UMoveToTowerProcessor()
{
	UE_LOG(LogTemp, Warning, TEXT("REGISTERED"));
	bAutoRegisterWithProcessingPhases = true;
	ExecutionFlags = (int32)EProcessorExecutionFlags::All;
	ExecutionOrder.ExecuteBefore.Add(UE::Mass::ProcessorGroupNames::Avoidance);
}

void UMoveToTowerProcessor::ConfigureQueries()
{
	myEntities.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadWrite);
	myEntities.AddRequirement<FMoveToTargetFragment>(EMassFragmentAccess::ReadWrite);

	myEntities.RegisterWithProcessor(*this);
}

void UMoveToTowerProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	//UE_LOG(LogTemp, Warning, TEXT("EXECUTED"));

	myEntities.ForEachEntityChunk(EntityManager, Context, [this](FMassExecutionContext& Context)
		{
			const TArrayView<FTransformFragment> transforms = Context.GetMutableFragmentView<FTransformFragment>();
			const TArrayView<FMoveToTargetFragment> targets = Context.GetMutableFragmentView<FMoveToTargetFragment>();

			for (int i = 0; i < Context.GetNumEntities(); ++i)
			{
				FTransform& transform = transforms[i].GetMutableTransform();
				const auto& targetPos = targets[i].myTarget;
				
				const auto& target = targetPos - transform.GetLocation();

				if (target.Length() < 200.f)
				{
					//UE_LOG(LogTemp, Warning, TEXT("ARRIVED"));

					//EntityManager->

					//EntityManager->Defer().DestroyEntity(Entity);
				}
				else
				{
					transform.SetLocation(transform.GetLocation() + target.GetSafeNormal() * 400.f * Context.GetDeltaTimeSeconds());
					//UE_LOG(LogTemp, Warning, TEXT("MOVING"));
				}

			}

		});
}
