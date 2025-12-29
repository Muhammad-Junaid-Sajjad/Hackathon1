#!/usr/bin/env python3
"""
Populate 84 section files with frontmatter from spec.md
"""
import re
from pathlib import Path

# Section metadata extracted from spec.md
sections = {
    # Module 1 - Chapter 1
    "M1/C1/S1": {"title": "Workstation Setup and Ubuntu 22.04 Configuration", "keywords": ["ubuntu", "nvidia", "workstation", "setup"]},
    "M1/C1/S2": {"title": "Jetson Orin Edge Kit Assembly and Flashing", "keywords": ["jetson", "edge", "flashing", "hardware"]},
    "M1/C1/S3": {"title": "Physical AI Principles and Embodied Intelligence", "keywords": ["physical-ai", "embodied", "intelligence", "principles"]},
    "M1/C1/S4": {"title": "ROS 2 Humble Installation and Workspace Configuration", "keywords": ["ros2", "humble", "installation", "workspace"]},
    "M1/C1/S5": {"title": "ROS 2 Nodes and Topics", "keywords": ["ros2", "nodes", "topics", "pub-sub"]},
    "M1/C1/S6": {"title": "Transform Trees (TF2) and Coordinate Frames", "keywords": ["tf2", "transforms", "coordinate-frames", "kinematics"]},
    "M1/C1/S7": {"title": "ROS 2 Command-Line Tools", "keywords": ["ros2-cli", "tools", "debugging", "introspection"]},

    # Module 1 - Chapter 2
    "M1/C2/S1": {"title": "ROS 2 Services and Request-Response Patterns", "keywords": ["services", "request-response", "synchronous"]},
    "M1/C2/S2": {"title": "ROS 2 Actions and Long-Running Tasks", "keywords": ["actions", "async", "feedback", "preemption"]},
    "M1/C2/S3": {"title": "ROS 2 Parameters and Dynamic Reconfiguration", "keywords": ["parameters", "config", "dynamic-reconfigure"]},
    "M1/C2/S4": {"title": "Launch Files and Multi-Node Orchestration", "keywords": ["launch", "orchestration", "multi-node"]},
    "M1/C2/S5": {"title": "Custom Message Interfaces", "keywords": ["messages", "interfaces", "custom-types"]},
    "M1/C2/S6": {"title": "Quality of Service (QoS) Policies", "keywords": ["qos", "reliability", "durability", "liveliness"]},
    "M1/C2/S7": {"title": "Rosbag Recording and Playback", "keywords": ["rosbag", "recording", "playback", "debugging"]},

    # Module 1 - Chapter 3
    "M1/C3/S1": {"title": "URDF Basics and Humanoid Robot Description", "keywords": ["urdf", "robot-description", "links", "joints"]},
    "M1/C3/S2": {"title": "Xacro Macros for Modular Robot Models", "keywords": ["xacro", "macros", "modular", "urdf"]},
    "M1/C3/S3": {"title": "Forward Kinematics and Joint State Publishing", "keywords": ["forward-kinematics", "joint-states", "kinematics"]},
    "M1/C3/S4": {"title": "Inverse Kinematics for End-Effector Control", "keywords": ["inverse-kinematics", "ik", "end-effector", "planning"]},
    "M1/C3/S5": {"title": "IMU Integration and Sensor Fusion", "keywords": ["imu", "sensor-fusion", "orientation", "filtering"]},
    "M1/C3/S6": {"title": "Collision Detection and Safety Boundaries", "keywords": ["collision", "safety", "boundaries", "protection"]},
    "M1/C3/S7": {"title": "Module 1 Consistency Check", "keywords": ["assessment", "module-check", "ros2", "validation"]},

    # Module 2 - Chapter 1
    "M2/C1/S1": {"title": "Gazebo Installation and URDF Spawning", "keywords": ["gazebo", "installation", "urdf", "simulation"]},
    "M2/C1/S2": {"title": "Physics Engines and Simulation Parameters", "keywords": ["physics", "ode", "bullet", "parameters"]},
    "M2/C1/S3": {"title": "World Files and Environmental Design", "keywords": ["world-files", "environment", "design", "scenes"]},
    "M2/C1/S4": {"title": "Contact Forces and Collision Properties", "keywords": ["contact", "collision", "forces", "friction"]},
    "M2/C1/S5": {"title": "Friction Models and Ground Interaction", "keywords": ["friction", "ground", "interaction", "physics"]},
    "M2/C1/S6": {"title": "Joint Controllers and Actuation", "keywords": ["controllers", "actuation", "pid", "joints"]},
    "M2/C1/S7": {"title": "ROS 2-Gazebo Bridge", "keywords": ["gazebo-ros", "bridge", "integration", "middleware"]},

    # Module 2 - Chapter 2
    "M2/C2/S1": {"title": "RealSense Camera Simulation", "keywords": ["realsense", "camera", "depth", "simulation"]},
    "M2/C2/S2": {"title": "LiDAR Sensors in Gazebo", "keywords": ["lidar", "laser", "scanning", "gazebo"]},
    "M2/C2/S3": {"title": "Depth Camera Calibration", "keywords": ["calibration", "depth", "camera", "intrinsics"]},
    "M2/C2/S4": {"title": "IMU Noise Models and Filtering", "keywords": ["imu", "noise", "filtering", "simulation"]},
    "M2/C2/S5": {"title": "RGB-D Alignment and Point Clouds", "keywords": ["rgbd", "point-clouds", "alignment", "3d"]},
    "M2/C2/S6": {"title": "Latency Simulation in Sensor Pipelines", "keywords": ["latency", "sensors", "pipeline", "delay"]},
    "M2/C2/S7": {"title": "Synthetic Data Generation for Training", "keywords": ["synthetic-data", "training", "dataset", "generation"]},

    # Module 2 - Chapter 3
    "M2/C3/S1": {"title": "Unity Installation and HDRP Setup", "keywords": ["unity", "hdrp", "installation", "rendering"]},
    "M2/C3/S2": {"title": "Unity-ROS Bridge Configuration", "keywords": ["unity-ros", "bridge", "tcp", "integration"]},
    "M2/C3/S3": {"title": "Humanoid Avatar Rigging", "keywords": ["avatar", "rigging", "humanoid", "animation"]},
    "M2/C3/S4": {"title": "Voice UI and Dialogue Systems", "keywords": ["voice-ui", "dialogue", "interaction", "speech"]},
    "M2/C3/S5": {"title": "Gesture Recognition", "keywords": ["gesture", "recognition", "motion", "tracking"]},
    "M2/C3/S6": {"title": "Domain Randomization Techniques", "keywords": ["randomization", "domain", "sim2real", "generalization"]},
    "M2/C3/S7": {"title": "Module 2 Consistency Check", "keywords": ["assessment", "module-check", "simulation", "validation"]},

    # Module 3 - Chapter 1
    "M3/C1/S1": {"title": "NVIDIA Omniverse Installation", "keywords": ["omniverse", "isaac-sim", "installation", "nvidia"]},
    "M3/C1/S2": {"title": "USD Format and Scene Composition", "keywords": ["usd", "scene", "composition", "format"]},
    "M3/C1/S3": {"title": "URDF to USD Conversion", "keywords": ["urdf", "usd", "conversion", "import"]},
    "M3/C1/S4": {"title": "Isaac Sim Physics Engine", "keywords": ["physx", "physics", "isaac-sim", "simulation"]},
    "M3/C1/S5": {"title": "Isaac Replicator for Synthetic Data", "keywords": ["replicator", "synthetic-data", "generation", "isaac"]},
    "M3/C1/S6": {"title": "OmniGraph for Visual Scripting", "keywords": ["omnigraph", "visual-scripting", "nodes", "workflow"]},
    "M3/C1/S7": {"title": "Cloud Rendering with Omniverse Farm", "keywords": ["cloud", "rendering", "omniverse-farm", "ether-lab"]},

    # Module 3 - Chapter 2
    "M3/C2/S1": {"title": "Isaac ROS Perception Stack", "keywords": ["isaac-ros", "perception", "stack", "integration"]},
    "M3/C2/S2": {"title": "Visual SLAM with cuVSLAM", "keywords": ["vslam", "cuvslam", "localization", "mapping"]},
    "M3/C2/S3": {"title": "Object Detection with Isaac ROS", "keywords": ["object-detection", "isaac-ros", "dnn", "inference"]},
    "M3/C2/S4": {"title": "Depth Segmentation", "keywords": ["segmentation", "depth", "semantic", "instance"]},
    "M3/C2/S5": {"title": "Nav2 Integration for Path Planning", "keywords": ["nav2", "path-planning", "navigation", "autonomous"]},
    "M3/C2/S6": {"title": "Occupancy Mapping", "keywords": ["occupancy", "mapping", "costmap", "grid"]},
    "M3/C2/S7": {"title": "Semantic Scene Understanding", "keywords": ["semantic", "scene", "understanding", "reasoning"]},

    # Module 3 - Chapter 3
    "M3/C3/S1": {"title": "Bipedal Locomotion Fundamentals", "keywords": ["bipedal", "locomotion", "walking", "gait"]},
    "M3/C3/S2": {"title": "Isaac Gym Reinforcement Learning", "keywords": ["isaac-gym", "rl", "training", "parallel"]},
    "M3/C3/S3": {"title": "PPO Policy Training for Walking", "keywords": ["ppo", "policy", "training", "locomotion"]},
    "M3/C3/S4": {"title": "Sim-to-Real Randomization", "keywords": ["sim2real", "randomization", "transfer", "generalization"]},
    "M3/C3/S5": {"title": "ONNX Export for Deployment", "keywords": ["onnx", "export", "deployment", "inference"]},
    "M3/C3/S6": {"title": "Flashing Weights to Jetson", "keywords": ["weight-flashing", "jetson", "deployment", "latency"]},
    "M3/C3/S7": {"title": "Module 3 Consistency Check", "keywords": ["assessment", "module-check", "isaac", "validation"]},

    # Module 4 - Chapter 1
    "M4/C1/S1": {"title": "ReSpeaker Mic Array Setup", "keywords": ["respeaker", "microphone", "audio", "capture"]},
    "M4/C1/S2": {"title": "Whisper ASR Integration", "keywords": ["whisper", "asr", "speech-recognition", "transcription"]},
    "M4/C1/S3": {"title": "Natural Language Parsing", "keywords": ["nlp", "parsing", "intent", "language"]},
    "M4/C1/S4": {"title": "Multimodal Sensor Fusion", "keywords": ["multimodal", "fusion", "vision", "language"]},
    "M4/C1/S5": {"title": "Dialogue Management", "keywords": ["dialogue", "management", "context", "conversation"]},
    "M4/C1/S6": {"title": "Gesture and Pointing Detection", "keywords": ["gesture", "pointing", "detection", "multimodal"]},
    "M4/C1/S7": {"title": "Text-to-Speech Synthesis", "keywords": ["tts", "synthesis", "speech", "output"]},

    # Module 4 - Chapter 2
    "M4/C2/S1": {"title": "LLM Integration for Task Decomposition", "keywords": ["llm", "task-decomposition", "planning", "language"]},
    "M4/C2/S2": {"title": "Behavior Trees for Hierarchical Planning", "keywords": ["behavior-trees", "planning", "hierarchical", "control"]},
    "M4/C2/S3": {"title": "Scene Context and Affordances", "keywords": ["context", "affordances", "scene", "reasoning"]},
    "M4/C2/S4": {"title": "Language-to-Action Grounding", "keywords": ["grounding", "language", "action", "vla"]},
    "M4/C2/S5": {"title": "Error Handling and Replanning", "keywords": ["error-handling", "replanning", "recovery", "robustness"]},
    "M4/C2/S6": {"title": "Safety Constraints and Guardrails", "keywords": ["safety", "constraints", "guardrails", "ethics"]},
    "M4/C2/S7": {"title": "Hierarchical Task Networks", "keywords": ["htn", "hierarchical", "task", "planning"]},

    # Module 4 - Chapter 3
    "M4/C3/S1": {"title": "Capstone Part 1: Voice to Plan", "keywords": ["capstone", "voice", "planning", "integration"]},
    "M4/C3/S2": {"title": "Capstone Part 2: Plan to Navigate", "keywords": ["capstone", "navigation", "planning", "execution"]},
    "M4/C3/S3": {"title": "Capstone Part 3: Navigate to Manipulate", "keywords": ["capstone", "manipulation", "navigation", "pipeline"]},
    "M4/C3/S4": {"title": "Capstone Simulation Testing", "keywords": ["capstone", "testing", "simulation", "validation"]},
    "M4/C3/S5": {"title": "Optional Physical Deployment", "keywords": ["physical", "deployment", "robot", "real-world"]},
    "M4/C3/S6": {"title": "Capstone Assessment Rubric", "keywords": ["assessment", "rubric", "grading", "evaluation"]},
    "M4/C3/S7": {"title": "Final Module Check and Assessment Matrix", "keywords": ["final-check", "assessment", "matrix", "completion"]},
}

def create_section_file(section_path: str, metadata: dict):
    """Create a section markdown file with frontmatter"""
    module, chapter, section = section_path.split('/')

    # Calculate sidebar position (1-7 for sections)
    section_num = int(section[1])

    content = f"""---
id: {section_path.replace('/', '-').lower()}
title: {metadata['title']}
sidebar_position: {section_num}
keywords: {metadata['keywords']}
---

# {metadata['title']}

*Coming soon...*

## Overview

This section is part of the Physical AI & Humanoid Robotics textbook.

## Prerequisites

- Completion of previous sections in this chapter

## Learning Objectives

*To be defined*

## Content

*Content will be added during the writing phase*

## Next Steps

Proceed to the next section in this chapter.
"""

    file_path = Path(f"docs/{section_path}.md")
    file_path.write_text(content)
    print(f"✓ Created {section_path}.md")

def main():
    """Populate all 84 section files"""
    print("Populating 84 section files with frontmatter...")
    print()

    for section_path, metadata in sections.items():
        create_section_file(section_path, metadata)

    print()
    print(f"✓ Successfully created {len(sections)} section files")

if __name__ == "__main__":
    main()
