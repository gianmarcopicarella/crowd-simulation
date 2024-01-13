// Fill out your copyright notice in the Description page of Project Settings.


#include "NavMeshMovementProcessor.h"

#include "AITypes.h"
#include "MassCommonFragments.h"
#include "MassEntityTemplateRegistry.h"
#include "MassMovementFragments.h"
#include "MassNavigationFragments.h"
#include "NavigationSystem.h"
#include "NavMeshPathTrait.h"
#include "Misc/App.h"
#include "MassExecutionContext.h"
#include "MassEntityManager.h"



UNavMeshMovementProcessor::UNavMeshMovementProcessor()
{
	UE_LOG(LogTemp, Warning, TEXT("NAVMESH REGISTERED"));
	bAutoRegisterWithProcessingPhases = true;
	ExecutionFlags = (int32)EProcessorExecutionFlags::All;
	ExecutionOrder.ExecuteBefore.Add(UE::Mass::ProcessorGroupNames::Avoidance);
}

void UNavMeshMovementProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	//int32 size = myEntities.GetNumMatchingEntities(EntityManager);
	//UE_LOG(LogTemp, Warning, TEXT("COUNT: %i"), size);
	//ParallelFor(size, [&](const int32 JobIndex)
	//{
	//Context.GetWorld()->GetDeltaSeconds();
	FApp::GetDeltaTime();
		
	myEntities.ForEachEntityChunk(EntityManager, Context, [this](FMassExecutionContext& Context)
		{
			const TArrayView<FTransformFragment> transforms = Context.GetMutableFragmentView<FTransformFragment>();
			const TArrayView<FMassMoveTargetFragment> NavTargetsList = Context.GetMutableFragmentView<FMassMoveTargetFragment>();
			const TArrayView<FNavMeshPathFragment> PathList = Context.GetMutableFragmentView<FNavMeshPathFragment>();
			const FMassMovementParameters& MovementParameters = Context.GetConstSharedFragment<FMassMovementParameters>();

			UNavigationSystemV1* NavSysPtr = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
			//UE_ASSUME(NavSysPtr != nullptr);
			//UE_LOG(LogTemp, Warning, TEXT("NAVMESH EXEC"));

			for (int32 EntityIndex = 0; EntityIndex < Context.GetNumEntities(); ++EntityIndex)
			{
				// Get fragment data
				const FTransform& Transform = transforms[EntityIndex].GetTransform(); /*Mutable*/
				FMassMoveTargetFragment& MoveTarget = NavTargetsList[EntityIndex];
				FNavMeshPathFragment& Path = PathList[EntityIndex];

				// TEMPORARY HACK
				if (Path.myHasMoved) continue;
				//
				// Get info from fragments
				FVector CurrentLocation = Transform.GetLocation();
				FVector& TargetVector = Path.myTargetLocation;

				// Calculate path

				if (Path.myPathResult.Result != ENavigationQueryResult::Success)
				{
					FNavLocation Result;

					NavSysPtr->GetRandomReachablePointInRadius(FVector{ 300,0,10 }, 2500.f, Result);
					//UE_LOG(LogTemp, Error, TEXT("Current Destination: %s"), *(Result.Location.ToString()));
					Path.myTargetLocation = Result.Location;

					FAIMoveRequest MoveRequest(TargetVector);
					const ANavigationData* NavData = NavSysPtr->GetDefaultNavDataInstance();
					FSharedConstNavQueryFilter NavFilter = UNavigationQueryFilter::GetQueryFilter(*NavData, this, MoveRequest.GetNavigationFilter());
					FPathFindingQuery Query(&*this, *NavData, CurrentLocation, TargetVector, NavFilter);
					Path.myPathResult = NavSysPtr->FindPathSync(Query);
					if (Path.myPathResult.Result == ENavigationQueryResult::Success)
					{
						Path.myPathIndex = 0;
						MoveTarget.SlackRadius = 100.f;
						MoveTarget.Center = Path.myPathResult.Path->GetPathPointLocation(1).Position;

						FHitResult OutHit;
						GetWorld()->LineTraceSingleByChannel(OutHit, Transform.GetLocation() + (Transform.GetRotation().GetUpVector() * 100), Transform.GetLocation() - (Transform.GetRotation().GetUpVector() * 100), ECollisionChannel::ECC_Visibility);
						MoveTarget.Center.Z = OutHit.ImpactPoint.Z;
					}

					MoveTarget.DesiredSpeed = FMassInt16Real(MovementParameters.DefaultDesiredSpeed);
				}

				MoveTarget.DistanceToGoal = (MoveTarget.Center - Transform.GetLocation()).Size();
				MoveTarget.Forward = (MoveTarget.Center - Transform.GetLocation()).GetSafeNormal();
				//DrawDebugDirectionalArrow(GetWorld(), Transform.GetLocation(), Transform.GetLocation() + MoveTarget.Forward * 15.f, 1.f, FColor::Red);

				/*Transform.SetLocation(Transform.GetLocation() +
					MoveTarget.Forward * MovementParameters.DefaultDesiredSpeed * Context.GetDeltaTimeSeconds());*/

					// Go from point to point to reach final destination
				if (MoveTarget.DistanceToGoal <= MoveTarget.SlackRadius)
				{
					if (Path.myPathIndex < Path.myPathResult.Path->GetPathPoints().Num() - 1)
					{
						++Path.myPathIndex;
						MoveTarget.Center = Path.myPathResult.Path->GetPathPoints()[Path.myPathIndex];

						FHitResult OutHit;
						GetWorld()->LineTraceSingleByChannel(OutHit, Transform.GetLocation() + (Transform.GetRotation().GetUpVector() * 100), Transform.GetLocation() - (Transform.GetRotation().GetUpVector() * 100), ECollisionChannel::ECC_Visibility);
						MoveTarget.Center.Z = OutHit.ImpactPoint.Z;
					}
					else
					{
						// TEMPORARY HACK
						Path.myHasMoved = true;

						Path.myPathIndex = 0;
						Path.myPathResult.Result = ENavigationQueryResult::Invalid;

						//UE_LOG(LogTemp, Error, TEXT("Current Destination: %s"), *(Path.TargetLocation.ToString()));
					}
				}
			}

		});

	//});
	
//
//
//
//	/*
//	myEntities.ForEachEntityChunk(EntitySubsystem, Context, ([this](FMassExecutionContext& Context)
//		{
//			const TConstArrayView<FTransformFragment> TransformsList = Context.GetFragmentView<FTransformFragment>();
//			const TArrayView<FMassMoveTargetFragment> NavTargetsList = Context.GetMutableFragmentView<FMassMoveTargetFragment>();
//			const TArrayView<FNavMeshPathFragment> PathList = Context.GetMutableFragmentView<FNavMeshPathFragment>();
//			const FMassMovementParameters& MovementParameters = Context.GetConstSharedFragment<FMassMovementParameters>();
//
//			UNavigationSystemV1* NavSys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
//
//			if (NavSys)
//			{
//				for (int32 EntityIndex = 0; EntityIndex < Context.GetNumEntities(); ++EntityIndex)
//				{
//					// Get fragment data
//					const FTransform& Transform = TransformsList[EntityIndex].GetTransform();
//					FMassMoveTargetFragment& MoveTarget = NavTargetsList[EntityIndex];
//					FNavMeshPathFragment& Path = PathList[EntityIndex];
//					int8& PathIndex = Path.PathIndex;
//
//					// Get info from fragments
//					FVector CurrentLocation = Transform.GetLocation();
//					FVector& TargetVector = Path.TargetLocation;
//
//					// Calculate path
//					if (Path.PathResult.Result != ENavigationQueryResult::Success)
//					{
//						FNavLocation Result;
//						NavSys->GetRandomReachablePointInRadius(Transform.GetLocation(), 3000.f, Result);
//						Path.TargetLocation = Result.Location;
//
//						FAIMoveRequest MoveRequest(TargetVector);
//						const ANavigationData* NavData = NavSys->GetDefaultNavDataInstance();
//						FSharedConstNavQueryFilter NavFilter = UNavigationQueryFilter::GetQueryFilter(*NavData, this, MoveRequest.GetNavigationFilter());
//						FPathFindingQuery Query(&*this, *NavData, CurrentLocation, TargetVector, NavFilter);
//						Path.PathResult = NavSys->FindPathSync(Query);
//						if (Path.PathResult.Result == ENavigationQueryResult::Success)
//						{
//							PathIndex = 0;
//							MoveTarget.SlackRadius = 100.f;
//							MoveTarget.Center = Path.PathResult.Path->GetPathPointLocation(1).Position;
//						}
//
//						// Use result to move ai
//
//						//DrawDebugLine(GetWorld(), Transform.GetLocation(),MoveTarget.Center, FColor::Red, true, -1, 0, 5);
//						MoveTarget.DesiredSpeed = FMassInt16Real(MovementParameters.DefaultDesiredSpeed);
//					}
//					MoveTarget.DistanceToGoal = (MoveTarget.Center - Transform.GetLocation()).Size();
//					MoveTarget.Forward = (MoveTarget.Center - Transform.GetLocation()).GetSafeNormal();
//					FHitResult OutHit;
//					GetWorld()->LineTraceSingleByChannel(OutHit, Transform.GetLocation() + (Transform.GetRotation().GetUpVector() * 100), Transform.GetLocation() - (Transform.GetRotation().GetUpVector() * 100), ECollisionChannel::ECC_Visibility);
//					MoveTarget.Center.Z = OutHit.ImpactPoint.Z;
//
//					// Go from point to point to reach final destination
//					if (MoveTarget.DistanceToGoal <= MoveTarget.SlackRadius)
//					{
//						if (PathIndex < Path.PathResult.Path->GetPathPoints().Num() - 1)
//						{
//							PathIndex++;
//							MoveTarget.Center = Path.PathResult.Path->GetPathPoints()[PathIndex];
//						}
//						else
//						{
//							PathIndex = 0;
//							Path.PathResult.Result = ENavigationQueryResult::Invalid;
//							FNavLocation Result;
//							NavSys->GetRandomReachablePointInRadius(Transform.GetLocation(), 3000.f, Result);
//							Path.TargetLocation = Result.Location;
//							//UE_LOG(LogTemp, Error, TEXT("Current Destination: %s"), *(Path.TargetLocation.ToString()));
//						}
//					}
//				}
//			}
//		}));*/
}

void UNavMeshMovementProcessor::ConfigureQueries()
{
	myEntities.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadWrite);
	myEntities.AddRequirement<FMassMoveTargetFragment>(EMassFragmentAccess::ReadWrite);
	myEntities.AddRequirement<FNavMeshPathFragment>(EMassFragmentAccess::ReadWrite);
	myEntities.AddConstSharedRequirement<FMassMovementParameters>(EMassFragmentPresence::All);

	myEntities.RegisterWithProcessor(*this);
}
