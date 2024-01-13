// Fill out your copyright notice in the Description page of Project Settings.


#include "CustomNavigationProcessor.h"
#include "CustomMassObstacleAvoidanceTrait.h"
#include "CustomAvoidanceProcessor.h"

#include "MassCommonUtils.h"
#include "MassCommandBuffer.h"
#include "MassCommonFragments.h"
#include "MassNavigationFragments.h"
#include "MassNavigationSubsystem.h"
#include "MassSimulationLOD.h"
#include "MassMovementTypes.h"
#include "MassMovementFragments.h"
#include "MassEntityView.h"
#include "MassExecutionContext.h"
#include "Engine/World.h"

#define UNSAFE_FOR_MT 0
#define MOVEMENT_DEBUGDRAW 0	// Set to 1 to see heading debugdraw
//----------------------------------------------------------------------//
//  UMassNavigationObstacleGridProcessor
//----------------------------------------------------------------------//
UCustomMassNavigationObstacleGridProcessor::UCustomMassNavigationObstacleGridProcessor()
{
	ExecutionFlags = (int32)EProcessorExecutionFlags::All;
	ExecutionOrder.ExecuteAfter.Add(UE::Mass::ProcessorGroupNames::Movement);
	
}

void UCustomMassNavigationObstacleGridProcessor::Initialize(UObject& Owner)
{
	Super::Initialize(Owner);

	CustomNavigationSubsystem = UWorld::GetSubsystem<UCustomMassNavigationSubsystem>(Owner.GetWorld());
}

void UCustomMassNavigationObstacleGridProcessor::ConfigureQueries()
{
	FMassEntityQuery BaseEntityQuery;
	BaseEntityQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
	BaseEntityQuery.AddRequirement<FAgentRadiusFragment>(EMassFragmentAccess::ReadOnly);
	BaseEntityQuery.AddRequirement<FCustomMassNavigationObstacleGridCellLocationFragment>(EMassFragmentAccess::ReadWrite);
	BaseEntityQuery.AddSubsystemRequirement<UCustomMassNavigationSubsystem>(EMassFragmentAccess::ReadWrite);

	AddToGridEntityQuery = BaseEntityQuery;
	AddToGridEntityQuery.AddRequirement<FMassAvoidanceColliderFragment>(EMassFragmentAccess::ReadOnly, EMassFragmentPresence::Optional);
	AddToGridEntityQuery.AddTagRequirement<FMassOffLODTag>(EMassFragmentPresence::None);
	AddToGridEntityQuery.AddTagRequirement<FMassInNavigationObstacleGridTag>(EMassFragmentPresence::None);
	AddToGridEntityQuery.RegisterWithProcessor(*this);

	UpdateGridEntityQuery = BaseEntityQuery;
	UpdateGridEntityQuery.AddRequirement<FMassAvoidanceColliderFragment>(EMassFragmentAccess::ReadOnly, EMassFragmentPresence::Optional);
	UpdateGridEntityQuery.AddTagRequirement<FMassOffLODTag>(EMassFragmentPresence::None);
	UpdateGridEntityQuery.AddTagRequirement<FMassInNavigationObstacleGridTag>(EMassFragmentPresence::All);
	UpdateGridEntityQuery.RegisterWithProcessor(*this);

	RemoveFromGridEntityQuery = BaseEntityQuery;
	RemoveFromGridEntityQuery.AddTagRequirement<FMassOffLODTag>(EMassFragmentPresence::All);
	RemoveFromGridEntityQuery.AddTagRequirement<FMassInNavigationObstacleGridTag>(EMassFragmentPresence::All);
	RemoveFromGridEntityQuery.RegisterWithProcessor(*this);
}

void UCustomMassNavigationObstacleGridProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	auto& pc = CustomNavigationSubsystem->GetObstacleTreeMutable();
	pc.pts.clear();

	AddToGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &pc, &EntityManager](FMassExecutionContext& Context)
		{
			const int32 NumEntities = Context.GetNumEntities();
			TConstArrayView<FTransformFragment> LocationList = Context.GetFragmentView<FTransformFragment>();

			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				// Add to the grid
				const auto entity = Context.GetEntity(EntityIndex);
				const FVector NewPos = LocationList[EntityIndex].GetTransform().GetLocation();
				pc.pts.emplace_back(PointCloud2D::Point{ NewPos.X, NewPos.Y, entity });
				Context.Defer().AddTag<FMassInNavigationObstacleGridTag>(entity);
			}
		});

	UpdateGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &pc, &EntityManager](FMassExecutionContext& Context)
		{
			const int32 NumEntities = Context.GetNumEntities();
			TConstArrayView<FTransformFragment> LocationList = Context.GetFragmentView<FTransformFragment>();

			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				// Add to the grid
				const auto entity = Context.GetEntity(EntityIndex);
				const FVector NewPos = LocationList[EntityIndex].GetTransform().GetLocation();
				pc.pts.emplace_back(PointCloud2D::Point{ NewPos.X, NewPos.Y, entity });
			}
		});

	RemoveFromGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &EntityManager](FMassExecutionContext& Context)
		{
			const int32 NumEntities = Context.GetNumEntities();

			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				Context.Defer().RemoveTag<FMassInNavigationObstacleGridTag>(Context.GetEntity(EntityIndex));
			}
		});

	
	// can't be ParallelFor due to MovementSubsystem->GetGridMutable().Move not being thread-safe
	/*
	AddToGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &EntityManager](FMassExecutionContext& Context)
		{
			auto& HashGrid = Context.GetMutableSubsystemChecked<UCustomMassNavigationSubsystem>().GetObstacleGridMutable();
			const int32 NumEntities = Context.GetNumEntities();

			TConstArrayView<FTransformFragment> LocationList = Context.GetFragmentView<FTransformFragment>();
			TConstArrayView<FAgentRadiusFragment> RadiiList = Context.GetFragmentView<FAgentRadiusFragment>();
			TArrayView<FCustomMassNavigationObstacleGridCellLocationFragment> NavigationObstacleCellLocationList = Context.GetMutableFragmentView<FCustomMassNavigationObstacleGridCellLocationFragment>();
			const bool bHasColliderData = Context.GetFragmentView<FMassAvoidanceColliderFragment>().Num() > 0;

			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				// Add to the grid
				const FVector NewPos = LocationList[EntityIndex].GetTransform().GetLocation();
				const float Radius = RadiiList[EntityIndex].Radius;

				FMassNavigationObstacleItem ObstacleItem;
				ObstacleItem.Entity = Context.GetEntity(EntityIndex);
				ObstacleItem.ItemFlags |= bHasColliderData ? EMassNavigationObstacleFlags::HasColliderData : EMassNavigationObstacleFlags::None;

				const FBox NewBounds(NewPos - FVector(Radius, Radius, 0.f), NewPos + FVector(Radius, Radius, 0.f));
				NavigationObstacleCellLocationList[EntityIndex].CellLoc = HashGrid.Add(ObstacleItem, NewBounds);

				Context.Defer().AddTag<FMassInNavigationObstacleGridTag>(ObstacleItem.Entity);
			}
		});

	UpdateGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &EntityManager](FMassExecutionContext& Context)
		{
			auto& HashGrid = Context.GetMutableSubsystemChecked<UCustomMassNavigationSubsystem>().GetObstacleGridMutable();
			const int32 NumEntities = Context.GetNumEntities();

			TConstArrayView<FTransformFragment> LocationList = Context.GetFragmentView<FTransformFragment>();
			TConstArrayView<FAgentRadiusFragment> RadiiList = Context.GetFragmentView<FAgentRadiusFragment>();
			TArrayView<FCustomMassNavigationObstacleGridCellLocationFragment> NavigationObstacleCellLocationList = Context.GetMutableFragmentView<FCustomMassNavigationObstacleGridCellLocationFragment>();
			const bool bHasColliderData = Context.GetFragmentView<FMassAvoidanceColliderFragment>().Num() > 0;

			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				// Update position in grid
				const FVector NewPos = LocationList[EntityIndex].GetTransform().GetLocation();
				const float Radius = RadiiList[EntityIndex].Radius;
				FMassNavigationObstacleItem ObstacleItem;
				ObstacleItem.Entity = Context.GetEntity(EntityIndex);
				ObstacleItem.ItemFlags |= bHasColliderData ? EMassNavigationObstacleFlags::HasColliderData : EMassNavigationObstacleFlags::None;

				const FBox NewBounds(NewPos - FVector(Radius, Radius, 0.f), NewPos + FVector(Radius, Radius, 0.f));
				NavigationObstacleCellLocationList[EntityIndex].CellLoc = HashGrid.Move(ObstacleItem, NavigationObstacleCellLocationList[EntityIndex].CellLoc, NewBounds);

#if WITH_MASSGAMEPLAY_DEBUG && 0
				const FDebugContext BaseDebugContext(this, LogAvoidance, nullptr, ObstacleItem.Entity);
				if (DebugIsSelected(ObstacleItem.Entity))
				{
					FBox Box = MovementSubsystem->GetGridMutable().CalcCellBounds(AvoidanceObstacleCellLocationList[EntityIndex].CellLoc);
					Box.Max.Z += 200.f;
					DebugDrawBox(BaseDebugContext, Box, FColor::Yellow);
				}
#endif // WITH_MASSGAMEPLAY_DEBUG
		}
});

	RemoveFromGridEntityQuery.ForEachEntityChunk(EntityManager, Context, [this, &EntityManager](FMassExecutionContext& Context)
		{
			auto& HashGrid = Context.GetMutableSubsystemChecked<UCustomMassNavigationSubsystem>().GetObstacleGridMutable();
			const int32 NumEntities = Context.GetNumEntities();

			TArrayView<FCustomMassNavigationObstacleGridCellLocationFragment> AvoidanceObstacleCellLocationList = Context.GetMutableFragmentView<FCustomMassNavigationObstacleGridCellLocationFragment>();
			for (int32 EntityIndex = 0; EntityIndex < NumEntities; ++EntityIndex)
			{
				FMassNavigationObstacleItem ObstacleItem;
				ObstacleItem.Entity = Context.GetEntity(EntityIndex);
				HashGrid.Remove(ObstacleItem, AvoidanceObstacleCellLocationList[EntityIndex].CellLoc);
				AvoidanceObstacleCellLocationList[EntityIndex].CellLoc = FNavigationObstacleHashGrid2D::FCellLocation();
				//UE_LOG(LogTemp, Warning, TEXT("AAA"));
				Context.Defer().RemoveTag<FMassInNavigationObstacleGridTag>(ObstacleItem.Entity);
			}
		});
		*/
}

//----------------------------------------------------------------------//
//  UMassNavigationObstacleRemoverProcessor
//----------------------------------------------------------------------//
UCustomMassNavigationObstacleRemoverProcessor::UCustomMassNavigationObstacleRemoverProcessor()
	: EntityQuery(*this)
{
	ObservedType = FCustomMassNavigationObstacleGridCellLocationFragment::StaticStruct();
	Operation = EMassObservedOperation::Remove;
	ExecutionFlags = (int32)(EProcessorExecutionFlags::All);
}

void UCustomMassNavigationObstacleRemoverProcessor::ConfigureQueries()
{
	EntityQuery.AddRequirement<FCustomMassNavigationObstacleGridCellLocationFragment>(EMassFragmentAccess::ReadWrite);
	EntityQuery.AddSubsystemRequirement<UCustomMassNavigationSubsystem>(EMassFragmentAccess::ReadWrite);
}

void UCustomMassNavigationObstacleRemoverProcessor::Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context)
{
	EntityQuery.ForEachEntityChunk(EntityManager, Context, [this](FMassExecutionContext& Context)
		{
			auto& HashGrid = Context.GetMutableSubsystemChecked<UCustomMassNavigationSubsystem>().GetObstacleGridMutable();
			const int32 NumEntities = Context.GetNumEntities();
			const TArrayView<FCustomMassNavigationObstacleGridCellLocationFragment> AvoidanceObstacleCellLocationList = Context.GetMutableFragmentView<FCustomMassNavigationObstacleGridCellLocationFragment>();

			for (int32 i = 0; i < NumEntities; ++i)
			{
				FMassNavigationObstacleItem ObstacleItem;
				ObstacleItem.Entity = Context.GetEntity(i);
				HashGrid.Remove(ObstacleItem, AvoidanceObstacleCellLocationList[i].CellLoc);
			}
		});
}

#undef UNSAFE_FOR_MT