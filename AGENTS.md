# eltanin project instructions

eltanin is a non-ROS C++ library for 2D autonomous navigation.

Always respond to the user in Japanese.

## Goals

- Improve C++ design and implementation skill.
- Build reusable 2D navigation components.
- Keep core logic independent from ROS2, visualization, and simulation runtime.
- Prefer correctness, testability, and observability over premature abstraction.

## Dependency rules

Core modules may depend on:
- C++ standard library
- Eigen

Optional modules may depend on:
- Rerun
- matplotlib-cpp
- ROS2
- testing/benchmark libraries

Do not introduce ROS2, Rerun, matplotlib-cpp, or Python dependencies into core planning/control/map logic.

## Review expectations

When reviewing code, use the robotics-cpp-reviewer skill if available.

Prioritize:
- correctness
- API clarity
- ownership/lifetime
- coordinate conversion correctness
- testability
- performance in hot loops
- debug visibility

For learning tasks, do not produce full implementations first.
Prefer hints, review comments, minimal snippets, and exercises.
