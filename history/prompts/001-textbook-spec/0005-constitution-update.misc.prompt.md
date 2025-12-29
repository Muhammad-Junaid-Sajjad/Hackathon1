---
id: 0005
title: Constitution Update - Course Details Integration
stage: misc
date: 2025-12-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-textbook-spec
user: junaid
command: Update constitution with comprehensive course details
labels: ["constitution","course-details","hardware","modules"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - npm run build (SUCCESS)
---

## Prompt

Update constitution with the comprehensive course details provided:
- Detailed module descriptions (Module 1-4 with specific topics)
- Complete hardware specifications (Workstation, Edge Kit, Robot Lab, Cloud)
- Prices for all hardware components
- Architecture diagram for lab setup

## Response snapshot

Updated constitution with comprehensive course information:

**Article IV - Module Truth (Expanded)**:
- Added detailed module descriptions for each of the 4 modules
- Module 1: ROS 2 architecture, rclpy, URDF, packages, launch files
- Module 2: Gazebo/Unity simulation, SDF formats, sensors (LiDAR, Depth, IMU)
- Module 3: Isaac Sim, Isaac ROS, VSLAM, Nav2, RL for control
- Module 4: Whisper ASR, LLM planning, multi-modal interaction

**Article VI - Hardware Truth (Complete Rewrite)**:
- Three heavy computational loads identified (Physics Sim, Visual Perception, Gen AI)
- Workstation specs with minimum/ideal specifications and prices
- Edge Kit breakdown:
  - Jetson Orin Nano Super: ~$249
  - RealSense D435i: ~$349
  - ReSpeaker Mic Array: ~$69
  - Total: ~$700 per kit
- Three robot lab options (Proxy, Miniature, Premium)
- Cloud lab option (AWS g5.2xlarge ~$1.50/hour, ~$205/quarter)
- ASCII architecture diagram

**Version**: Updated to 1.2.0

## Outcome

- ✅ Impact: Constitution now contains complete course specifications
- 🧪 Tests: `npm run build` passes
- 📁 Files: Modified 1 file (.specify/memory/constitution.md)
- 🔁 Next prompts: Verify 84 sections align with new module descriptions
- 🧠 Reflection: Constitution now serves as comprehensive reference for course implementation

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): `npm run build` PASS
- Prompt variant (if applicable): N/A
- Next experiment (if needed): Review section alignment with updated module descriptions
