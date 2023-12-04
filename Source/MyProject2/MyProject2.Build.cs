// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

// Setup based on https://forums.unrealengine.com/t/unresolved-reference-fmassfragment/572465

public class MyProject2 : ModuleRules
{
	public MyProject2(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });
		PrivateDependencyModuleNames.AddRange(new string[] { "MassAIBehavior",
            "MassAIDebug",

			// Runtime/MassEntity Plugin Modules
			"MassEntity",

			// Runtime/MassGameplay Plugin Modules
			"MassActors",
            "MassCommon",
            "MassGameplayDebug",
            "MassLOD",
            "MassMovement",
            "MassNavigation",
            "MassRepresentation",
            "MassReplication",
            "MassSpawner",
            "MassSimulation",
            "MassSignals", });
    }
}