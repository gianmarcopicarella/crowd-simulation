// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"
#include "MassNavigationSubsystem.h"
#include "HierarchicalHashGrid2D.h"

#include "nanoflann.hpp"

#include "CustomAvoidanceProcessor.generated.h"

/*

*/

struct PointCloud2D
{
	struct Point
	{
		double x, y;
		FMassEntityHandle entity;
	};

	std::vector<Point> pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate
	// value, the
	//  "if/else's" are actually solved at compile time.
	inline double kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		if (dim == 0)
		{
			return pts[idx].x;
		}
		else
		{
			return pts[idx].y;
		}
	}

	// Optional bounding-box computation: return false to default to a standard
	// bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned
	//   in "bb" so it can be avoided to redo it again. Look at bb.size() to
	//   find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /* bb */) const
	{
		return false;
	}
};

using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<double, PointCloud2D>,
	PointCloud2D, 2
>;



typedef THierarchicalHashGrid2D<2, 4, FMassNavigationObstacleItem> CustomHashGrid2D;	// 2 levels of hierarchy, 4 ratio between levels

UCLASS()
class MYPROJECT2_API UCustomMassNavigationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()

public:
	UCustomMassNavigationSubsystem();

	virtual void Initialize(FSubsystemCollectionBase& Collection) override;

	const CustomHashGrid2D& GetObstacleGrid() const { return CustomAvoidanceObstacleGrid; }
	CustomHashGrid2D& GetObstacleGridMutable() { return CustomAvoidanceObstacleGrid; }

	const PointCloud2D& GetObstacleTree() const { return points; }
	PointCloud2D& GetObstacleTreeMutable() { return points; }

protected:

	// Same as standard implementation for now
	CustomHashGrid2D CustomAvoidanceObstacleGrid;



	// Test structure
	PointCloud2D points;
};

template<>
struct TMassExternalSubsystemTraits<UCustomMassNavigationSubsystem> final
{
	enum
	{
		GameThreadOnly = false
	};
};

/**
 *
 */
UCLASS()
class MYPROJECT2_API UCustomAvoidanceProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
	UCustomAvoidanceProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Initialize(UObject& Owner) override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

private:
	TObjectPtr<UWorld> World;
	TObjectPtr<UCustomMassNavigationSubsystem> CustomNavigationSubsystem;
	FMassEntityQuery EntityQuery;

};


/**
 *
 */
UCLASS()
class MYPROJECT2_API UCustomStandingAvoidanceProcessor : public UMassProcessor
{
	GENERATED_BODY()

public:
	UCustomStandingAvoidanceProcessor();

protected:
	virtual void ConfigureQueries() override;
	virtual void Initialize(UObject& Owner) override;
	virtual void Execute(FMassEntityManager& EntityManager, FMassExecutionContext& Context) override;

private:
	TObjectPtr<UWorld> World;
	TObjectPtr<UCustomMassNavigationSubsystem> CustomNavigationSubsystem;
	FMassEntityQuery EntityQuery;

};
