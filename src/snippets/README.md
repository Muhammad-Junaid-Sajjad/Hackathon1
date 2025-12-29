# Code Snippets Directory

This directory contains Python code examples and ROS 2 configurations referenced throughout the Physical AI & Humanoid Robotics textbook.

## Structure

- `ros2/` - ROS 2 Humble code examples (nodes, launch files, URDF/Xacro files)
- `gazebo/` - Gazebo simulation configurations and plugins
- `isaac/` - NVIDIA Isaac Sim and Isaac ROS examples
- `vla/` - Vision-Language-Action integration code (Whisper, LLM, behavior trees)

## Usage

These snippets are referenced from the main documentation files using code blocks with file paths. Students can download individual files or clone the entire repository to access all code examples.

## File Naming Convention

Files follow the pattern: `Mx_Cy_Sz_description.ext`

Examples:
- `M1_C1_S1_workstation_setup.sh`
- `M1_C2_S2_action_server.py`
- `M2_C1_S1_spawn_urdf.launch.py`
- `M3_C2_S3_object_detection.py`
- `M4_C2_S4_language_grounding.py`

## Code Standards

All Python code follows:
- PEP 8 style guidelines
- Type hints for function signatures
- Docstrings for all public functions/classes
- ROS 2 Humble API conventions
- Inline comments for complex logic

All bash scripts include:
- Shebang line (`#!/bin/bash`)
- Error handling (`set -e`)
- Usage documentation
- Clear variable names
