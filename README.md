📌 Overview

This repository contains the complete firmware and control logic for an autonomous Maze Solver Robot developed for Technoxian.
The robot is designed to explore an unknown maze, map it efficiently, and compute the shortest path to the target using algorithmic decision-making and closed-loop motor control.

The project emphasizes real-world embedded system constraints, including sensor noise, timing accuracy, memory limitations, and precise motion control.

**Algorithms Used**

**Flood Fill Algorithm** for maze mapping and optimal path planning

**PID Control** for:

Straight-line motion

Accurate 90° turns

Drift correction during high-speed traversal

These algorithms are implemented keeping hardware-in-the-loop execution in mind rather than ideal simulations.

**Key Features**

Fully autonomous operation

Real-time maze discovery and mapping

Adaptive path optimization

Modular and scalable firmware structure

Competition-oriented design with focus on reliability and speed
