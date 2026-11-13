#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>

// Random obstacle placement for the testbed's scenes (random_cylinders.xml and
// g1_random_cylinders.xml). Pass applyCylinderPlacements to MotionPlanner as
// its SceneEditor, and the viewer applies it too, so all of them see the same
// positions for the whole run.

// Reseed the placement stream and clear this run's cached positions.
// enabled = false keeps the scene XML's placements.
void seedSceneRandomization(std::uint32_t seed, bool enabled = true);

// Move the run's cylinders to their random positions. Scenes without them are
// left unchanged. Randomization can be turned off with the toggle at the top
// of testbed/src/scene_randomizer.cpp.
void applyCylinderPlacements(mjModel* model);
