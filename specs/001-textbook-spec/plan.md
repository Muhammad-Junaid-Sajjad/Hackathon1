# Implementation Plan: Physical AI & Humanoid Robotics Textbook (13-Week Execution Roadmap)

**Branch**: `001-textbook-spec` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-textbook-spec/spec.md`

---

## Summary

Create a comprehensive Docusaurus textbook with 84 sections (4 modules × 3 chapters × 7 sections) covering Physical AI & Humanoid Robotics. The textbook bridges the gap between digital intelligence and physical embodiment through a simulation-first, sim-to-real workflow. Content spans ROS 2 middleware, Digital Twin simulation (Gazebo & Unity), NVIDIA Isaac AI platform, and Vision-Language-Action integration, culminating in a capstone project where a simulated humanoid executes voice commands. Execution follows a 13-week schedule with 6 major milestones, ensuring compliance with the Constitution's 4×3×7 structure, scope lock (brief-only tools), narrative spine (Embodied Intelligence → Digital Twin → Sim-to-Real), and latency trap enforcement (train on workstation/cloud → flash to Jetson for inference).

---

## Technical Context

**Language/Version**: Markdown (Docusaurus-flavored), Python 3.11+ (for code examples), Bash (for setup scripts)

**Primary Dependencies**:
- **Static Site Generator**: Docusaurus (latest stable: v3.x)
- **Deployment**: GitHub Pages + GitHub Actions CI/CD
- **Content Tools**: MDX (for interactive components), Prism (syntax highlighting)
- **Stack Documentation**: ROS 2 Humble, Gazebo Harmonic, Unity 2022.3 LTS, NVIDIA Isaac Sim 4.x, Jetson JetPack 6.x

**Storage**: Git repository with Markdown files; no database required (static site)

**Testing**:
- **Build Validation**: `npm run build` (Docusaurus build check for broken links, invalid MDX)
- **Link Checking**: `markdown-link-check` (validate internal/external links)
- **Structure Validation**: Custom bash script to verify 4×3×7 directory schema (`docs/M[1-4]/C[1-3]/S[1-7].md`)

**Target Platform**: GitHub Pages (static hosting), target browsers: Chrome/Firefox/Safari (latest 2 versions), mobile-responsive

**Project Type**: Documentation site (Docusaurus static site generation)

**Performance Goals**:
- Build time: <5 minutes (84 Markdown files + assets)
- Page load time: <3 seconds (first contentful paint)
- Search indexing: <2 seconds for local search results

**Constraints**:
- **Scope Lock** (Constitution Article II): Only tools from brief (ROS 2, Gazebo, Unity, Isaac, Whisper, Jetson, specified robots)
- **4×3×7 Structure** (Constitution Article III): Exactly 84 sections, no deviation
- **Latency Trap Rule** (Constitution Article VII): 36 placements (22 "Required Callout" + 14 "Mention") must be present
- **Information Density** (Constitution Article X): Every section requires ≥1 runnable code artifact or technical procedure

**Scale/Scope**: 84 sections, estimated 200-400 lines per section (total: 16,800-33,600 lines of technical Markdown), ~50-100 code examples, ~30-50 configuration files, ~20-30 diagrams

---

## Constitution Check

*GATE: Must pass before implementation. Violations require explicit justification.*

**✅ Article I (Mission)**: Textbook bridges digital brain ↔ physical body via simulation-first workflow with voice-commanded humanoid capstone.

**✅ Article II (Scope Lock)**: Specification references ONLY brief-approved tools (ROS 2 Humble, Gazebo Harmonic, Unity, Isaac Sim/Lab/ROS, RealSense D435i/D455, Whisper, Jetson Orin, Unitree G1/Go2, Robotis OP3, AWS g5.2xlarge). No external dependencies introduced.

**✅ Article III (4×3×7 Structure)**: Plan produces exactly 84 sections in `docs/M[1-4]/C[1-3]/S[1-7].md` structure. Module Consistency Checks at M1/M2/M3/M4-C3-S7 act as completion gates.

**✅ Article IV (Module Truth)**: Module names immutable: M1 (ROS 2 Nervous System), M2 (Digital Twin), M3 (NVIDIA Isaac™), M4 (Vision-Language-Action).

**✅ Article V (Narrative Thread)**: Every section includes "Connection to Capstone" subsection linking to Voice → Plan → Actions → Navigate → Vision → Manipulation pipeline.

**✅ Article VI (Hardware Truth)**: Hardware specs documented in M1-C1-S1 (Workstation: RTX 4070 Ti+), M1-C1-S2 (Jetson Orin), with Economy/Proxy/Premium hardware tables.

**✅ Article VII (Latency Trap Rule)**: 36 placements enforced (22 "Required Callout" in M1-C1-S2, M3-C2-S2/S3, M3-C3-S3/S6, M4-C1-S2, M4-C3-S5, Module Checks + 14 "Mention" in M1-C2-S1, M1-C2-S6, M1-C3-S4, M2-C2-S6, M3-C1-S5, M3-C1-S7). Standard callout pattern provided in Clarification Report (spec.md Section F).

**✅ Article VIII (Capstone Definition)**: M4-C3 (Chapter 3) provides full Voice → Plan → Navigate → Manipulate pipeline. M4-C3-S4 confirms simulation-first (Gazebo + Isaac Sim). M4-C3-S5 marks physical deployment as "Optional."

**✅ Article IX (Delivery Toolchain)**: Docusaurus + GitHub Pages + Spec-Kit Plus workflow + Claude Code agent.

**✅ Article X (Information Density)**: Success Criteria SC-003 enforces ≥1 runnable code artifact per section. "No fluff" rule: conceptual sections (e.g., M1-C1-S3 Physical AI Principles) must include technical artifacts (comparison table, Python pseudocode, annotated diagrams).

**✅ Article XI (Repository Rules)**: Directory schema `docs/M[1-4]/C[1-3]/S[1-7].md` enforced. Sidebar auto-generated from directory structure. CI validation via bash script.

**✅ Article XII (Assessment Alignment)**: Section E (spec.md lines 487-533) maps all 6 assessments to section IDs. M4-C3-S7 includes "Assessment Alignment Matrix" table.

**✅ Article XIII (Hackathon Bonus Scope)**: RAG chatbot, authentication, personalization, Urdu translation are OUT OF SCOPE for 84 sections. Bonus features implemented as optional Docusaurus plugins post-completion.

**No Constitution Violations Detected**. Proceed with implementation.

---

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-spec/
├── spec.md              # Technical specification (84-section map, hardware tables, assessment alignment)
├── plan.md              # This file: 13-week execution roadmap
├── clarification.md     # Constitution audit report (embedded in spec.md Section F)
└── tasks.md             # Task breakdown (generated via /sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
/
├── docs/                           # Docusaurus content (4×3×7 structure)
│   ├── M1/                        # Module 1: ROS 2 Nervous System
│   │   ├── C1/                    # Chapter 1: Foundations & Hardware
│   │   │   ├── S1.md              # Workstation Setup
│   │   │   ├── S2.md              # Jetson Edge Kit
│   │   │   ├── S3.md              # Physical AI Principles
│   │   │   ├── S4.md              # ROS 2 Installation
│   │   │   ├── S5.md              # Nodes/Topics/Pub-Sub
│   │   │   ├── S6.md              # TF2 Fundamentals
│   │   │   └── S7.md              # CLI Tools & Debugging
│   │   ├── C2/                    # Chapter 2: ROS 2 Logic Layer
│   │   │   ├── S1.md              # Services
│   │   │   ├── S2.md              # Actions
│   │   │   ├── S3.md              # Parameters
│   │   │   ├── S4.md              # Launch Files
│   │   │   ├── S5.md              # Custom Messages
│   │   │   ├── S6.md              # QoS Policies
│   │   │   └── S7.md              # Rosbag Recording
│   │   └── C3/                    # Chapter 3: Humanoid URDF/TF2
│   │       ├── S1.md              # URDF Basics
│   │       ├── S2.md              # Xacro Macros
│   │       ├── S3.md              # Forward Kinematics
│   │       ├── S4.md              # Inverse Kinematics
│   │       ├── S5.md              # IMU Integration
│   │       ├── S6.md              # Collision Geometries
│   │       └── S7.md              # Module 1 Consistency Check
│   ├── M2/                        # Module 2: Digital Twin (21 sections, structure mirrors M1)
│   ├── M3/                        # Module 3: NVIDIA Isaac™ (21 sections)
│   └── M4/                        # Module 4: Vision-Language-Action (21 sections)
│
├── static/                         # Static assets
│   ├── img/                       # Diagrams, hardware photos, architecture diagrams
│   ├── code/                      # Downloadable code examples (Python scripts, configs)
│   └── datasets/                  # Sample datasets (synthetic data examples)
│
├── src/                            # Docusaurus source (theme customizations, plugins)
│   ├── components/                # MDX components (e.g., LatencyTrapCallout.jsx)
│   └── css/                       # Custom styling
│
├── sidebars.js                     # Auto-generated sidebar reflecting 4×3×7 structure
├── docusaurus.config.js            # Docusaurus configuration (title, GitHub Pages URL, plugins)
├── package.json                    # Node dependencies (Docusaurus, MDX, plugins)
│
├── .github/
│   └── workflows/
│       └── deploy.yml             # GitHub Actions: build + deploy to GitHub Pages
│
├── scripts/                        # Validation scripts
│   ├── validate-structure.sh     # Verify 4×3×7 directory schema exists
│   ├── validate-latency-traps.sh # Count latency trap placements (must = 36)
│   └── validate-links.sh         # Check for broken internal links
│
└── README.md                       # Repository documentation (setup instructions)
```

**Structure Decision**: Docusaurus documentation site with strict 4×3×7 content hierarchy (`docs/M[1-4]/C[1-3]/S[1-7].md`). No backend/frontend split required—purely static content generation. CI/CD via GitHub Actions validates structure, builds site, deploys to GitHub Pages.

---

## Complexity Tracking

> **No Constitution violations detected**. No complexity tracking required.

---

## 1) Milestones (Brief-Aligned)

### Milestone 0: Repository + Docusaurus Scaffold Ready
**Week**: 1
**Deliverables**:
- Docusaurus project initialized (`npx create-docusaurus@latest`)
- Directory structure `docs/M[1-4]/C[1-3]/` created (84 placeholder files)
- `sidebars.js` configured to reflect 4×3×7 hierarchy
- GitHub Actions workflow (`deploy.yml`) configured for GitHub Pages deployment
- Validation scripts (`validate-structure.sh`, `validate-latency-traps.sh`) created
- Initial build passes (`npm run build`)

**Success Criteria**: `npm run build` succeeds; `scripts/validate-structure.sh` confirms 84 files exist at correct paths.

---

### Milestone 1: Module 1 Complete (21 Sections + Consistency Check)
**Weeks**: 2-3
**Deliverables**:
- **M1-C1**: 7 sections (S1-S7) written with hardware setup guides, ROS 2 basics, TF2 fundamentals
- **M1-C2**: 7 sections (S1-S7) written with ROS 2 logic layer (Services, Actions, Parameters, Launch, QoS)
- **M1-C3**: 7 sections (S1-S7) written with URDF/TF2, kinematics, IMU integration, Module 1 Consistency Check
- All sections include: "Connection to Capstone" subsection, ≥1 code artifact, hardware requirements (where applicable), latency trap callouts (M1-C1-S2: Required; M1-C2-S1/S6, M1-C3-S4: Mention)
- Cross-link validation: URDF defined in M1-C3-S1 (canonical path `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf`)
- **M1-C3-S7** (Consistency Check) validates ROS 2 functionality, URDF kinematics, IMU integration

**Success Criteria**: 21 files exist at `docs/M1/C[1-3]/S[1-7].md`; M1-C3-S7 checklist passed; Assessment 1 (ROS 2 Quiz) preparable from M1-C1-S4/S5/S6/S7, M1-C2-S1/S3/S4.

---

### Milestone 2: Module 2 Complete (21 Sections + Consistency Check)
**Weeks**: 4-6
**Deliverables**:
- **M2-C1**: 7 sections (S1-S7) written with Gazebo physics (world setup, engines, collisions, controllers, ROS 2 bridge)
- **M2-C2**: 7 sections (S1-S7) written with sensor simulation (RealSense, LiDAR, IMU noise, synthetic data)
- **M2-C3**: 7 sections (S1-S7) written with Unity HRI (installation, rendering, ROS 2 bridge, domain randomization, Module 2 Consistency Check)
- All sections include latency trap callouts (M2-C2-S6: Mention)
- Cross-link validation: M2-C1-S1 references M1-C3-S1 URDF (spawning in Gazebo); M2-C2-S7 synthetic data flows to M3-C2-S3 (object detection training)
- **M2-C3-S7** (Consistency Check) validates Gazebo physics, sensor accuracy, Unity-ROS bridge functionality

**Success Criteria**: 21 files exist at `docs/M2/C[1-3]/S[1-7].md`; M2-C3-S7 checklist passed; Assessments 2 (URDF & Simulation Lab) and 3 (Sensor Simulation) preparable.

---

### Milestone 3: Module 3 Complete (21 Sections + Consistency Check)
**Weeks**: 7-9
**Deliverables**:
- **M3-C1**: 7 sections (S1-S7) written with Omniverse/USD (Nucleus setup, USD fundamentals, URDF→USD conversion, Isaac physics, Replicator, OmniGraph, Cloud rendering)
- **M3-C2**: 7 sections (S1-S7) written with Isaac ROS perception (installation, cuVSLAM, object detection, segmentation, Nav2, mapping)
- **M3-C3**: 7 sections (S1-S7) written with Sim-to-Real/RL (bipedal locomotion, Isaac Gym, PPO training, domain randomization, ONNX export, weight flashing, Module 3 Consistency Check)
- All sections include latency trap callouts (M3-C1-S7, M3-C2-S2/S3, M3-C3-S3/S6: Required; M3-C1-S5: Mention)
- Cross-link validation: M3-C1-S3 references M1-C3-S1 URDF (USD conversion); M3-C3-S6 documents SCP/USB weight flashing (PRIMARY REFERENCE for latency trap)
- **M3-C3-S7** (Consistency Check) validates Isaac Sim setup, VSLAM accuracy, RL policy transfer to Jetson

**Success Criteria**: 21 files exist at `docs/M3/C[1-3]/S[1-7].md`; M3-C3-S7 checklist passed; Assessments 4 (Isaac Sim & RL) and 5 (Edge Deployment & Nav2) preparable; Ether Lab ($205/quarter) documented in M3-C1-S7, M3-C3-S2/S3.

---

### Milestone 4: Module 4 Complete + Capstone Finalized (21 Sections + Consistency Check)
**Weeks**: 10-12
**Deliverables**:
- **M4-C1**: 7 sections (S1-S7) written with multimodal perception (ReSpeaker setup, Whisper ASR, command parsing, multimodal fusion, dialogue, gestures, TTS)
- **M4-C2**: 7 sections (S1-S7) written with cognitive planning (LLM integration, behavior trees, context windows, language→action grounding, error handling, safety validation, hierarchical planning)
- **M4-C3**: 7 sections (S1-S7) written with capstone integration (Voice→Plan, Plan→Navigate, Navigate→Manipulate, simulation testing, physical deployment [Optional], evaluation rubric, Module 4 + Final Consistency Check)
- All sections include latency trap callouts (M4-C1-S2, M4-C3-S5: Required)
- Cross-link validation: M4-C1-S2 (Whisper) → M4-C1-S3 (parsing) → M4-C2-S4 (grounding) → M1-C2-S2 (Actions); M4-C3-S2 references M3-C2-S5 (Nav2)
- **M4-C3-S7** (Final Consistency Check + Assessment Alignment Matrix) validates complete capstone pipeline (Voice → Plan → Navigate → Manipulate) achievable in simulation

**Success Criteria**: 21 files exist at `docs/M4/C[1-3]/S[1-7].md`; M4-C3-S7 includes Assessment Alignment Matrix mapping all 6 assessments to section IDs; Assessment 6 (VLA & Capstone Demo) preparable; Capstone fully achievable in Gazebo + Isaac Sim (physical deployment optional).

---

### Milestone 5: GitHub Pages Deployment Complete
**Week**: 13
**Deliverables**:
- All 84 sections finalized (no [TODO] placeholders remain)
- Docusaurus build passes without warnings (`npm run build`)
- All validation scripts pass:
  - `validate-structure.sh`: 84 files at correct paths
  - `validate-latency-traps.sh`: 36 placements confirmed (22 Required + 14 Mention)
  - `validate-links.sh`: No broken internal links
- GitHub Actions workflow deploys site to GitHub Pages
- Site accessible at `https://<username>.github.io/<repo-name>/`
- README.md includes setup instructions for students (hardware requirements, software install)

**Success Criteria**: Site live on GitHub Pages; all 84 sections accessible; sidebar navigation reflects 4×3×7 structure; search functionality operational; mobile-responsive.

---

## 2) Week-by-Week Plan (Weeks 1–13)

### Week 1: Docusaurus Scaffolding & Infrastructure Setup

**Theme**: Repository initialization, Article XI folder creation, sidebar wiring.

**Sections to Draft**: None (infrastructure week)

**Output of the Week**:
- Docusaurus project initialized:
  - `npx create-docusaurus@latest physical-ai-textbook classic --typescript`
  - `package.json`, `docusaurus.config.js`, `sidebars.js` configured
- Directory structure created:
  - `mkdir -p docs/M{1..4}/C{1..3}` (12 chapter directories)
  - `touch docs/M{1..4}/C{1..3}/S{1..7}.md` (84 placeholder files)
- `sidebars.js` configured with 4-level hierarchy:
  ```javascript
  module.exports = {
    tutorialSidebar: [
      {type: 'category', label: 'M1: ROS 2 Nervous System', items: [
        {type: 'category', label: 'C1: Foundations & Hardware', items: [
          'M1/C1/S1', 'M1/C1/S2', ..., 'M1/C1/S7'
        ]},
        // ... C2, C3
      ]},
      // ... M2, M3, M4
    ]
  };
  ```
- GitHub Actions workflow created (`.github/workflows/deploy.yml`):
  - Trigger: push to `main` branch
  - Steps: `npm install` → `npm run build` → deploy to `gh-pages` branch
- Validation scripts created (`scripts/validate-structure.sh`, `validate-latency-traps.sh`, `validate-links.sh`)
- Initial build test: `npm run build` (succeeds with empty sections)

**Quality Checks**:
- ✅ Structure count: 84 files exist at `docs/M[1-4]/C[1-3]/S[1-7].md`
- ✅ Sidebar navigation: 4 modules × 3 chapters × 7 sections visible
- ✅ Build passes: No errors, only warnings for empty files

**Capstone Progression**: Infrastructure ready for content authoring.

---

### Week 2: Module 1 - Chapter 1 (Foundations & Hardware)

**Theme**: Workstation setup, Jetson flashing, ROS 2 basics.

**Sections to Draft**: M1-C1-S1, M1-C1-S2, M1-C1-S3, M1-C1-S4, M1-C1-S5, M1-C1-S6, M1-C1-S7 (7 sections)

**Output of the Week**:
- **M1-C1-S1** (Workstation Setup): Ubuntu 22.04 install, NVIDIA driver setup, CUDA verification. Artifacts: `setup-workstation.sh`, `test-gpu.sh`.
- **M1-C1-S2** (Jetson Edge Kit): JetPack 6.x flashing, peripheral connection (RealSense, IMU, ReSpeaker), `lsusb` verification. **Latency Trap: Required Callout**. Artifacts: `flash-jetson.sh`, `test-peripherals.sh`. Hardware table: Economy Kit ($700).
- **M1-C1-S3** (Physical AI Principles): Embodied vs. disembodied AI, physics constraints. Artifacts: Comparison table (Markdown), `physics-aware-planning.py` (pseudocode), humanoid sensor-actuator diagram (SVG).
- **M1-C1-S4** (ROS 2 Installation): ROS 2 Humble apt install, colcon workspace setup, `.bashrc` sourcing. Artifacts: `install-ros2.sh`, `~/ros2_ws` directory tree.
- **M1-C1-S5** (Nodes/Topics/Pub-Sub): ROS 2 graph explanation, publisher/subscriber implementation. Artifacts: `joint_cmd_publisher.py`, `sensor_listener.py`, `rqt_graph` screenshot.
- **M1-C1-S6** (TF2 Fundamentals): Coordinate frame transforms, static transform broadcaster, RViz2 visualization. Artifacts: `broadcast_static_tf.py`, `rviz2_config.rviz`, TF tree diagram.
- **M1-C1-S7** (CLI Tools): `ros2 topic`, `ros2 node`, `ros2 bag`, `ros2 doctor` usage. Artifacts: `ros2-cli-cheatsheet.md`, `record-rosbag.sh`, `ros2 doctor` output example.

**Quality Checks**:
- ✅ Structure count: 7 files written (cumulative: 7/84)
- ✅ Terminology consistency: Use "ROS 2 Humble" (not "ROS2" or "ROS 2 Foxy")
- ✅ Latency trap coverage: M1-C1-S2 includes "Required Callout" (1/22 completed)
- ✅ Hardware truth: M1-C1-S1 lists RTX 4070 Ti minimum; M1-C1-S2 documents Jetson Orin Nano/NX specs

**Capstone Progression**: Foundation infrastructure (ROS 2 middleware) for capstone communication backbone.

---

### Week 3: Module 1 - Chapters 2 & 3 (ROS 2 Logic + URDF/TF2)

**Theme**: ROS 2 Services/Actions/Parameters + Humanoid kinematics.

**Sections to Draft**: M1-C2-S1 through M1-C3-S7 (14 sections)

**Output of the Week**:

**Chapter 2 (ROS 2 Logic Layer)**:
- **M1-C2-S1** (Services): ROS 2 service server/client, IK service example. **Latency Trap: Mention** (avoid cloud services in real-time loops). Artifacts: `ik_service.py`, `ik_client.py`, `.srv` definition.
- **M1-C2-S2** (Actions): Long-running tasks, action server/client, NavigateToPose example. Artifacts: `navigate_action_server.py`, `.action` definition, feedback callback.
- **M1-C2-S3** (Parameters): Runtime parameter management, YAML configs. Artifacts: `param_node.py`, `config.yaml`, `ros2 param` commands.
- **M1-C2-S4** (Launch Files): Multi-node orchestration, Python launch files. Artifacts: `capstone_bringup.launch.py`, launch file tutorial.
- **M1-C2-S5** (Custom Messages): `.msg`/`.srv`/`.action` definitions, CMakeLists.txt rules. Artifacts: `HumanoidJointState.msg`, build instructions.
- **M1-C2-S6** (QoS Policies): Reliability vs. best-effort QoS for sensors/commands. **Latency Trap: Mention** (QoS affects latency). Artifacts: `qos_profiles.py`, QoS comparison table.
- **M1-C2-S7** (Rosbag): Recording/playback for debugging. Artifacts: `ros2 bag record` commands, compression options.

**Chapter 3 (Humanoid URDF/TF2)**:
- **M1-C3-S1** (URDF Basics): **CANONICAL URDF DEFINITION** (`~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf`). Artifacts: `humanoid_arm.urdf`, XML tag explanations, RViz2 visualization.
- **M1-C3-S2** (Xacro): Xacro macros for modular URDFs. Artifacts: `humanoid_torso.xacro`, `xacro` CLI conversion.
- **M1-C3-S3** (Forward Kinematics): FK computation, `robot_state_publisher`. Artifacts: `joint_state_publisher.py`, FK script.
- **M1-C3-S4** (Inverse Kinematics): IK solver (KDL/TracIK), ROS 2 service wrapper. **Latency Trap: Mention** (IK on workstation for planning). Artifacts: `ik_solver.py`.
- **M1-C3-S5** (IMU Integration): USB IMU connection to Jetson, `robot_localization` EKF. Artifacts: `imu_driver.py`, EKF config YAML.
- **M1-C3-S6** (Collision Geometries): Collision meshes, MoveIt2 collision checking. Artifacts: URDF with `<collision>` tags, STL meshes.
- **M1-C3-S7** (Module 1 Consistency Check): ROS 2 tests, URDF validation, IMU streaming. **Latency Trap: Required Callout**. Artifacts: Checklist table, `test-module1.sh`, RViz2 screenshot.

**Quality Checks**:
- ✅ Structure count: 14 files written (cumulative: 21/84)
- ✅ Terminology consistency: "URDF" (not "urdf"), "TF2" (not "tf2"), "Jetson Orin Nano" (not "Jetson Nano")
- ✅ Latency trap coverage: M1-C2-S1/S6, M1-C3-S4 (Mention: 3/14 completed), M1-C3-S7 (Required: 2/22 completed)
- ✅ Cross-link validation: M1-C3-S1 URDF canonical path documented; future M2/M3 sections reference this

**Capstone Progression**: ROS 2 Actions (M1-C2-S2) enable capstone navigation/manipulation actions. URDF (M1-C3-S1) defines humanoid structure for simulation.

**Module 1 Completion Gate**: M1-C3-S7 checklist must pass before proceeding to Module 2. Freeze M1 content (no further edits unless critical bugs found).

---

### Week 4: Module 2 - Chapter 1 (Gazebo Physics)

**Theme**: Gazebo Harmonic installation, world building, physics configuration.

**Sections to Draft**: M2-C1-S1 through M2-C1-S7 (7 sections)

**Output of the Week**:
- **M2-C1-S1** (Gazebo Installation): Gazebo Harmonic install, spawn humanoid URDF from M1-C3-S1. **Cross-link: References M1-C3-S1 URDF path**. Artifacts: `install-gazebo.sh`, `.world` file, launch command.
- **M2-C1-S2** (Physics Engines): ODE/Bullet/DART comparison, solver tuning. Artifacts: SDF physics config, timestep tuning guide, stability test.
- **M2-C1-S3** (World Building): Terrain (stairs, ramps), obstacles (walls, boxes), props (blocks). Artifacts: SDF world file, Blender mesh export, `gz model` CLI.
- **M2-C1-S4** (Collision Detection): Contact surface parameters (friction, bounce), contact sensors. Artifacts: SDF collision config, contact force test.
- **M2-C1-S5** (Friction Models): Static/dynamic/rolling friction, foot-ground interaction. Artifacts: SDF friction params, Coulomb friction explanation, slip detection.
- **M2-C1-S6** (Joint Controllers): PID controllers via `ros2_control`, actuator limits. Artifacts: `ros2_control` YAML, PID tuning script, effort/velocity plots.
- **M2-C1-S7** (Gazebo ROS 2 Bridge): `ros_gz_bridge` topic mapping. Artifacts: Bridge config YAML, topic remapping, integrated launch file.

**Quality Checks**:
- ✅ Structure count: 7 files written (cumulative: 28/84)
- ✅ Cross-link validation: M2-C1-S1 references `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf` (no URDF duplication)
- ✅ Terminology: "Gazebo Harmonic" (not "Gazebo 11" or "Ignition"), "SDF" (not "sdf")

**Capstone Progression**: Gazebo provides simulation environment for testing humanoid navigation (M4-C3-S2) and manipulation (M4-C3-S3).

---

### Week 5: Module 2 - Chapter 2 (Sensor Simulation)

**Theme**: RealSense, LiDAR, IMU simulation, synthetic data generation.

**Sections to Draft**: M2-C2-S1 through M2-C2-S7 (7 sections)

**Output of the Week**:
- **M2-C2-S1** (RealSense D435i Simulation): Depth camera plugin, point cloud publisher. Artifacts: URDF camera link with Gazebo plugin, RViz2 depth visualization.
- **M2-C2-S2** (LiDAR Simulation): 2D/3D LiDAR, PCL filtering. Artifacts: URDF LiDAR link, ray sensor plugin, `pcl_filter.py`.
- **M2-C2-S3** (Depth Calibration): Camera intrinsics, `camera_calibration` procedure. Artifacts: `camera_info` YAML, calibration checkerboard guide.
- **M2-C2-S4** (IMU Noise Models): Gaussian noise, bias drift in Gazebo IMU plugin. **Cross-link: Distinct from M1-C3-S5 (real hardware)**. Artifacts: Gazebo IMU noise config, EKF tuning.
- **M2-C2-S5** (RGB-D Alignment): Align RGB+depth streams, depth hole filling. Artifacts: `align_rgbd.py`, depth accuracy validation plot.
- **M2-C2-S6** (Simulating Latency): Artificial latency/packet loss for robustness testing. **Latency Trap: Mention** (simulation zero-latency vs. real edge latency). Artifacts: `delay_node.py`, latency histogram.
- **M2-C2-S7** (Synthetic Data): Capture RGB-D images/labels for object detection training. **Cross-link: Feeds M3-C2-S3 (training)**. Artifacts: `capture_synthetic_data.py`, COCO-format annotations.

**Quality Checks**:
- ✅ Structure count: 7 files written (cumulative: 35/84)
- ✅ Latency trap coverage: M2-C2-S6 (Mention: 4/14 completed)
- ✅ Cross-link validation: M2-C2-S4 (simulated IMU) vs. M1-C3-S5 (real IMU) distinction documented; M2-C2-S7 synthetic data links to M3-C2-S3

**Capstone Progression**: Sensor simulation enables capstone perception (object detection for "red block") and navigation (obstacle detection via LiDAR/depth).

**Assessment 2 Preparation**: M1-C3-S1/S2/S3/S4, M2-C1-S1/S3/S7 prepare URDF & Simulation Lab assessment.

---

### Week 6: Module 2 - Chapter 3 (Unity & HRI) + Module 2 Consistency Check

**Theme**: Unity installation, high-fidelity rendering, Unity-ROS bridge, domain randomization.

**Sections to Draft**: M2-C3-S1 through M2-C3-S7 (7 sections)

**Output of the Week**:
- **M2-C3-S1** (Unity Installation): Unity Editor, Unity Robotics Hub ROS 2 package, ROS TCP Connector. Artifacts: Unity project setup, ROS 2 pub/sub demo.
- **M2-C3-S2** (High-Fidelity Rendering): HDRP, global illumination, high-res textures. Artifacts: Unity HDRP scene, material library, rendered image comparison (Unity vs. Gazebo).
- **M2-C3-S3** (Unity-ROS Bridge): Stream Unity camera/IMU to ROS 2 topics. Artifacts: Unity sensor publisher scripts, ROS 2 subscriber verification.
- **M2-C3-S4** (Humanoid Avatar): Import humanoid model, rig with Unity Avatar, play `JointState`-driven animations. Artifacts: Unity humanoid prefab, Avatar mapping, animation script.
- **M2-C3-S5** (Voice Command UI): Unity UI canvas for voice input, display task progress, connect to ROS 2 actions. Artifacts: Unity UI prefab, voice input handler, action client integration.
- **M2-C3-S6** (Domain Randomization): Randomize textures/lighting/object placements for sim-to-real transfer. Artifacts: Unity randomization script, randomized dataset export, perception accuracy comparison.
- **M2-C3-S7** (Module 2 Consistency Check): Gazebo physics tests, Unity-ROS bridge validation, sensor data comparison. **Latency Trap: Required Callout** (simulation zero-latency vs. edge). Artifacts: Checklist table, `test-module2.sh`, Gazebo+Unity screenshots.

**Quality Checks**:
- ✅ Structure count: 7 files written (cumulative: 42/84)
- ✅ Latency trap coverage: M2-C3-S7 (Required: 3/22 completed)
- ✅ Module 2 completion gate: M2-C3-S7 checklist passes (Gazebo worlds functional, Unity-ROS bridge operational)

**Capstone Progression**: Unity HRI (M2-C3-S5) provides voice command interface for capstone demo. Domain randomization (M2-C3-S6) improves sim-to-real transfer for capstone perception.

**Module 2 Completion Gate**: M2-C3-S7 checklist must pass before proceeding to Module 3. Freeze M2 content.

**Assessment 3 Preparation**: M2-C2-S1 through S7 prepare Sensor Simulation & Perception assessment.

---

### Week 7: Module 3 - Chapter 1 Part 1 (Omniverse & USD Start)

**Theme**: NVIDIA Omniverse installation, USD fundamentals, URDF→USD conversion.

**Sections to Draft**: M3-C1-S1 through M3-C1-S6 (6 sections) + M2-C3-S7 finalization

**Output of the Week**:
- **M3-C1-S1** (Omniverse Installation): Omniverse Launcher, Nucleus (local/cloud), Isaac Sim install, GPU verification. Artifacts: `install-omniverse.sh`, sample scene launch, GPU benchmark.
- **M3-C1-S2** (USD Fundamentals): USD prims/attributes/relationships, Python USD reader. Artifacts: USD reader script, sample USD file, prim hierarchy diagram.
- **M3-C1-S3** (URDF→USD Conversion): Convert M1-C3-S1 humanoid URDF to USD for Isaac Sim. **Cross-link: References M1-C3-S1 canonical URDF**. Artifacts: `urdf_to_usd.py`, Isaac Sim USD asset, physics test.
- **M3-C1-S4** (Isaac Physics): PhysX GPU physics, solver tuning for humanoid contact. Artifacts: Isaac Sim physics config, solver tuning table, GPU vs. CPU speedup benchmark.
- **M3-C1-S5** (Synthetic Data Replicator): Isaac Sim Replicator API for large-scale randomized datasets. **Latency Trap: Mention** (training on workstation/cloud). **Cross-link: Complements M2-C2-S7 Gazebo data**. Artifacts: Replicator script, randomized scene config, COCO dataset output.
- **M3-C1-S6** (OmniGraph ROS 2): Connect Isaac Sim to ROS 2 via OmniGraph nodes (pub/sub/actions). Artifacts: OmniGraph ROS 2 setup, action client integration, topic latency measurement.

**Quality Checks**:
- ✅ Structure count: 6 files written (cumulative: 48/84)
- ✅ Latency trap coverage: M3-C1-S5 (Mention: 5/14 completed)
- ✅ Cross-link validation: M3-C1-S3 references M1-C3-S1 URDF; M3-C1-S5 Replicator data links to M3-C2-S3 (object detection training)

**Capstone Progression**: Isaac Sim (M3-C1-S3/S4) provides high-fidelity physics simulation for capstone testing. Synthetic data (M3-C1-S5) trains object detectors for capstone vision.

---

### Week 8: Module 3 - Chapter 1 Part 2 + Chapter 2 Start (Isaac ROS Perception)

**Theme**: Cloud rendering (Ether Lab), Isaac ROS installation, cuVSLAM, object detection.

**Sections to Draft**: M3-C1-S7, M3-C2-S1 through M3-C2-S7 (8 sections) **[Doubling up week]**

**Output of the Week**:
- **M3-C1-S7** (Cloud Rendering): Omniverse Farm (cloud), Nucleus cloud storage, batch rendering. **Latency Trap: Required Callout** (cloud for training, NOT real-time control). **Ether Lab: $205/quarter breakdown ($1.20/hr × ~170 GPU-hrs)**. Artifacts: Farm submission script, Nucleus cloud config, cost estimation table.
- **M3-C2-S1** (Isaac ROS Installation): Install Isaac ROS GEMs (accelerated perception) on workstation + Jetson. Artifacts: Isaac ROS install script, GEM verification, GPU vs. CPU benchmark.
- **M3-C2-S2** (cuVSLAM): NVIDIA cuVSLAM for visual-inertial odometry on Jetson. **Latency Trap: Required Callout** (VSLAM runs on Jetson for real-time localization). Artifacts: cuVSLAM launch file, RealSense+IMU calibration, VSLAM trajectory plot.
- **M3-C2-S3** (Object Detection): TensorRT-optimized detection (YOLO/DetectNet) on Jetson. **Latency Trap: Required Callout** (training on workstation/cloud → TensorRT inference on Jetson). **Cross-link: Uses synthetic data from M2-C2-S7 + M3-C1-S5**. Artifacts: TensorRT conversion script, Isaac ROS DNN launch file, latency benchmark.
- **M3-C2-S4** (Depth Segmentation): RANSAC plane fitting for floor/object segmentation. Artifacts: `plane_fit_ransac.py`, segmented point cloud publisher, object centroid extraction.
- **M3-C2-S5** (Nav2 Integration): Configure ROS 2 Nav2 for humanoid navigation (costmaps, DWB planner, recovery). **Cross-link: M4-C3-S2 uses Nav2**. Artifacts: Nav2 params YAML, costmap config, behavior tree XML, Gazebo navigation test.
- **M3-C2-S6** (Occupancy Mapping): Build 2D occupancy grids with `slam_toolbox` for Nav2 localization. Artifacts: `slam_toolbox` config, map building script, saved map (PGM+YAML).
- **M3-C2-S7** (Semantic Segmentation): TensorRT semantic segmentation (ESANet/SegFormer) on Jetson for scene understanding. Artifacts: TensorRT model, Isaac ROS integration, segmented image visualization.

**Quality Checks**:
- ✅ Structure count: 8 files written (cumulative: 56/84)
- ✅ Latency trap coverage: M3-C1-S7, M3-C2-S2/S3 (Required: 6/22 completed); M3-C1-S5 (Mention: 5/14 completed)
- ✅ Ether Lab documentation: M3-C1-S7 includes $205/quarter cost breakdown, 170 GPU-hour budget, use cases (rendering, RL training, synthetic data)
- ✅ Cross-link validation: M3-C2-S3 references M2-C2-S7 + M3-C1-S5 for training data; M3-C2-S5 Nav2 documented for M4-C3-S2 use

**Capstone Progression**: cuVSLAM (M3-C2-S2) enables capstone obstacle navigation. Object detection (M3-C2-S3) enables "red block" detection. Nav2 (M3-C2-S5) provides navigation stack for capstone.

**Assessment 4 Preparation (Start)**: M3-C1-S1/S2/S3/S5 prepare Omniverse/USD/Isaac Gym foundations.

---

### Week 9: Module 3 - Chapter 3 (Sim-to-Real/RL)

**Theme**: Bipedal locomotion, Isaac Gym RL, PPO training, domain randomization, ONNX export, weight flashing.

**Sections to Draft**: M3-C3-S1 through M3-C3-S7 (7 sections)

**Output of the Week**:
- **M3-C3-S1** (Bipedal Locomotion): ZMP, COG, gait phases, open-loop walking controller. Artifacts: ZMP calculation script, gait trajectory planner, Gazebo walking demo, stability plot.
- **M3-C3-S2** (Isaac Gym Setup): Install Isaac Gym, create humanoid RL environment (state/action spaces), configure parallel simulation. **Latency Trap: Required Callout** (RL training on workstation/cloud). **Ether Lab: Cloud option for large-scale training**. Artifacts: Isaac Gym install script, custom environment code, state/action definitions, parallel env verification.
- **M3-C3-S3** (PPO Training): Train bipedal walking policy with PPO, monitor rewards (velocity, stability), hyperparameter tuning. **Latency Trap: Required Callout** (policy training on workstation/cloud). **Ether Lab: Cloud training option**. Artifacts: PPO training script, reward function, training curves, tensorboard logs, hyperparameter guide.
- **M3-C3-S4** (Domain Randomization): Randomize mass/friction/joint damping/sensor noise in Isaac Gym for sim-to-real robustness. Artifacts: Randomization config, before/after policy performance comparison, real-world transfer test.
- **M3-C3-S5** (ONNX Export): Export trained PyTorch policy to ONNX for TensorRT optimization. **Latency Trap: Required Callout** (ONNX model runs on Jetson for real-time inference). Artifacts: `pytorch_to_onnx.py`, TensorRT conversion script, latency benchmark, accuracy validation.
- **M3-C3-S6** (Flashing Weights to Jetson): **PRIMARY LATENCY TRAP REFERENCE**. Transfer TensorRT weights to Jetson via SCP/USB, load in ROS 2 node for 50+ Hz inference. **Latency Trap: Required Callout** (includes latency comparison table: cloud 50-200ms vs. Jetson <10ms). Artifacts: Model transfer script, ROS 2 TensorRT inference node, latency histogram, real-time control loop validation.
- **M3-C3-S7** (Module 3 Consistency Check): Isaac Sim tests, VSLAM accuracy, policy transfer to Jetson validation. **Latency Trap: Required Callout** (critical sim-to-real transition—verify workstation training → Jetson inference). Artifacts: Checklist table, `test-module3.sh`, Jetson inference latency report.

**Quality Checks**:
- ✅ Structure count: 7 files written (cumulative: 63/84)
- ✅ Latency trap coverage: M3-C3-S2/S3/S5/S6/S7 (Required: 11/22 completed)
- ✅ M3-C3-S6 is PRIMARY REFERENCE: All 14 "Mention" sections must link here for weight flashing procedure
- ✅ Ether Lab documentation: M3-C3-S2/S3 document cloud RL training option (~170 GPU-hours budget)

**Capstone Progression**: Bipedal locomotion (M3-C3-S1) enables humanoid walking for capstone navigation (M4-C3-S2). Weight flashing (M3-C3-S6) demonstrates sim-to-real workflow (train in simulation → deploy to Jetson).

**Module 3 Completion Gate**: M3-C3-S7 checklist must pass (Isaac Sim operational, VSLAM accurate, RL policy transferred to Jetson). Freeze M3 content.

**Assessment 4 Preparation (Complete)**: M3-C1-S1/S2/S3/S5, M3-C3-S2/S3/S4 prepare Isaac Sim & RL Training assessment.
**Assessment 5 Preparation (Start)**: M1-C1-S2, M3-C2-S1/S2/S3/S5, M3-C3-S6 prepare Edge Deployment & Nav2 assessment.

---

### Week 10: Module 4 - Chapter 1 Part 1 (Multimodal Perception Start)

**Theme**: ReSpeaker setup, Whisper ASR, command parsing, multimodal fusion, dialogue management.

**Sections to Draft**: M4-C1-S1 through M4-C1-S6 (6 sections) + M3-C3-S7 finalization

**Output of the Week**:
- **M4-C1-S1** (ReSpeaker Setup): Connect ReSpeaker 4/6-mic array to Jetson, configure ALSA, capture audio for VAD. Artifacts: `install-respeaker.sh`, ALSA config, `audio_capture.py`, VAD test script.
- **M4-C1-S2** (Whisper ASR): Deploy Whisper (tiny/base) on Jetson with TensorRT for real-time speech-to-text. **Latency Trap: Required Callout** (Whisper inference on Jetson; avoid cloud ASR for real-time control). Artifacts: `whisper_tensorrt.py`, ROS 2 ASR node, transcription accuracy test, latency benchmark.
- **M4-C1-S3** (Command Parsing): Parse Whisper transcriptions into structured commands (intent/entity extraction with regex/spaCy). **Cross-link: Feeds M4-C2-S4 (language→action grounding)**. Artifacts: `command_parser.py`, intent/entity examples, ROS 2 command publisher, test cases.
- **M4-C1-S4** (Multimodal Fusion): Fuse voice commands with visual context (e.g., "that block" + detected red block bounding box). Artifacts: `multimodal_fusion.py`, spatial grounding algorithm, ROS 2 fused command publisher, test cases.
- **M4-C1-S5** (Dialogue Management): Dialogue manager requesting clarification when commands ambiguous (e.g., "Which block?"). Artifacts: Dialogue state machine, clarification prompt generator, ROS 2 dialogue action server, Unity HRI feedback integration.
- **M4-C1-S6** (Gesture Recognition): Detect hand gestures (pointing, waving) with MediaPipe on Jetson, combine with voice. Artifacts: MediaPipe TensorRT model, gesture detection ROS 2 node, gesture-voice fusion example, latency benchmark.

**Quality Checks**:
- ✅ Structure count: 6 files written (cumulative: 69/84)
- ✅ Latency trap coverage: M4-C1-S2 (Required: 12/22 completed)
- ✅ Cross-link validation: M4-C1-S3 (parsing) feeds M4-C2-S4 (grounding to ROS 2 Actions)

**Capstone Progression**: Whisper ASR (M4-C1-S2) + command parsing (M4-C1-S3) enable capstone voice input ("Bring me the red block"). Multimodal fusion (M4-C1-S4) resolves spatial references.

**Assessment 4 Completion**: M3-C1-S1/S2/S3/S5, M3-C3-S2/S3/S4 fully prepare Isaac Sim & RL Training assessment.

---

### Week 11: Module 4 - Chapter 1 Part 2 + Chapter 2 (Cognitive Planning)

**Theme**: Voice feedback (TTS), LLM integration, behavior trees, context windows, language→action grounding, error handling, safety validation.

**Sections to Draft**: M4-C1-S7, M4-C2-S1 through M4-C2-S7 (8 sections) **[Doubling up week]**

**Output of the Week**:
- **M4-C1-S7** (Voice Feedback TTS): Generate voice feedback ("Navigating to the block") with piper-TTS on Jetson. Artifacts: `tts_deployment.sh`, ROS 2 TTS node, audio playback via ReSpeaker, latency benchmark.
- **M4-C2-S1** (LLM Integration): Deploy Llama 3.2-3B (Jetson 8GB) or 8B (Jetson 16GB) for task decomposition ("Bring me the red block" → navigate, grasp, return). Artifacts: LLM inference script (TensorRT/ONNX), ROS 2 task planner node, decomposition examples, latency benchmark. Note: Cloud LLM fallback for non-reactive planning (50-200ms latency acceptable).
- **M4-C2-S2** (Behavior Trees): Implement BTs with `py_trees` to execute task sequences (navigate→detect→grasp→return), handle failures. **Cross-link: Integrates Nav2 (M3-C2-S5) + manipulation**. Artifacts: `behavior_tree.py`, ROS 2 action client integration, BT visualization, failure recovery examples.
- **M4-C2-S3** (Context Windows): Manage LLM context (limited tokens) by summarizing dialogue history and environment state. Artifacts: Context manager implementation, sliding window algorithm, token budget monitoring, multi-turn dialogue example.
- **M4-C2-S4** (Language→Action Grounding): Map LLM subtasks (e.g., "navigate to object") to ROS 2 action calls (`NavigateToPose`, `GraspObject`). **Cross-link: Triggers M1-C2-S2 action servers**. Artifacts: Grounding mapping table, ROS 2 action dispatcher, parameter extraction (e.g., object color from NLU).
- **M4-C2-S5** (Error Handling): Detect action failures (nav timeout, grasp failure), trigger LLM replanning or recovery behaviors. Artifacts: Error detection logic, replanning trigger conditions, LLM replan prompt, recovery behavior examples.
- **M4-C2-S6** (Safe Action Validation): Validate LLM-generated actions against safety constraints (collision-free paths, reachable poses). Artifacts: Safety validator node, MoveIt2 collision checking integration, validation test cases, rejection examples.
- **M4-C2-S7** (Hierarchical Planning): Implement HTN for multi-step tasks (e.g., "clean the table" → locate→grasp→place in bin). Artifacts: HTN planner implementation, hierarchical task definition, multi-step execution example.

**Quality Checks**:
- ✅ Structure count: 8 files written (cumulative: 77/84)
- ✅ Cross-link validation: M4-C2-S2 references M3-C2-S5 (Nav2); M4-C2-S4 triggers M1-C2-S2 (ROS 2 Actions)
- ✅ Terminology: "Llama 3.2-3B" (not "Llama3"), "py_trees" (not "PyTrees"), "HTN" (Hierarchical Task Network)

**Capstone Progression**: LLM task decomposition (M4-C2-S1) + behavior trees (M4-C2-S2) + language→action grounding (M4-C2-S4) enable capstone cognitive planning layer (Voice → Plan → Actions).

**Assessment 5 Completion**: M1-C1-S2, M3-C2-S1/S2/S3/S5, M3-C3-S6 fully prepare Edge Deployment & Nav2 assessment.

---

### Week 12: Module 4 - Chapter 3 (Capstone Integration)

**Theme**: Voice→Plan, Plan→Navigate, Navigate→Manipulate integration, simulation testing, physical deployment (optional), evaluation rubric.

**Sections to Draft**: M4-C3-S1 through M4-C3-S6 (6 sections)

**Output of the Week**:
- **M4-C3-S1** (Voice→Plan Integration): Integrate Whisper ASR (M4-C1-S2) with LLM task decomposition (M4-C2-S1). **Cross-link: M4-C1-S2 → M4-C1-S3 → M4-C2-S1**. Artifacts: Integrated launch file, example commands ("Bring the red block"), task plan output, ROS 2 topic flow diagram.
- **M4-C3-S2** (Plan→Navigate Integration): Connect task planner to Nav2 navigation actions. **Cross-link: Uses M3-C2-S5 Nav2**. Artifacts: Behavior tree integrating planner + Nav2, navigation success criteria, obstacle avoidance test, Gazebo/Isaac Sim demo video.
- **M4-C3-S3** (Navigate→Manipulate Integration): Transition from navigation to manipulation—detect object with Isaac ROS (M3-C2-S3), compute grasp pose with MoveIt2, execute grasp action. Artifacts: Object detection + grasp planning integration, MoveIt2 grasp execution, success/failure metrics, simulation + real-world demo.
- **M4-C3-S4** (Capstone Simulation Testing): Test complete Voice→Plan→Navigate→Manipulate pipeline in Gazebo + Isaac Sim. Validate diverse scenarios (varied object positions, obstacles). **Simulation-First Rule: Capstone fully achievable without physical hardware**. Artifacts: Capstone test worlds, success rate metrics, failure mode analysis, simulation video recordings.
- **M4-C3-S5** (Capstone Physical Deployment): **OPTIONAL** Deploy to real humanoid (Unitree G1, Robotis OP3) with Jetson Orin. **Latency Trap: Required Callout** (simulation zero-latency vs. real sensor/actuator delays 10-50ms). Artifacts: Physical deployment checklist, calibration procedures (camera, IMU), real-world test results, sim-vs-real comparison.
- **M4-C3-S6** (Capstone Evaluation Rubric): Define evaluation criteria (success rate, completion time, failure recovery, code quality). Artifacts: Grading rubric table, success rate calculation, video submission requirements, code documentation standards.

**Quality Checks**:
- ✅ Structure count: 6 files written (cumulative: 83/84)
- ✅ Latency trap coverage: M4-C3-S5 (Required: 13/22 completed)
- ✅ Cross-link validation: M4-C3-S1 (Voice→Plan: M4-C1-S2/S3 → M4-C2-S1), M4-C3-S2 (Plan→Navigate: M4-C2-S2 → M3-C2-S5), M4-C3-S3 (Navigate→Manipulate: M3-C2-S3 object detection)
- ✅ Simulation-First Rule: M4-C3-S4 confirms capstone fully achievable in Gazebo + Isaac Sim; M4-C3-S5 physical deployment marked "Optional"

**Capstone Progression**: M4-C3-S1/S2/S3 integrate complete pipeline (Voice → Plan → Navigate → Manipulate). M4-C3-S4 validates in simulation. M4-C3-S5 (optional) deploys to physical robot.

**Assessment 6 Preparation (Start)**: M4-C1-S1/S2/S3, M4-C2-S1/S2/S4, M4-C3-S1/S2/S3/S4 prepare VLA Integration & Capstone Demo assessment.

---

### Week 13: Module 4 - Final Consistency Check + GitHub Pages Deployment

**Theme**: Final Assessment Alignment Matrix, Module 4 Consistency Check, GitHub Pages deployment, validation.

**Sections to Draft**: M4-C3-S7 (1 section)

**Output of the Week**:
- **M4-C3-S7** (Module 4 + Final Consistency Check): Verify complete capstone pipeline functional (Voice → Plan → Navigate → Manipulate → Sim-to-Real). **Assessment Alignment Matrix** (Constitution Article XII): Map all 6 assessments to section IDs (Assessment 1: M1-C1-S4/S5/S6/S7, M1-C2-S1/S3/S4; Assessment 2: M1-C3-S1/S2/S3/S4, M2-C1-S1/S3/S7; ... Assessment 6: M4-C1-S1/S2/S3, M4-C2-S1/S2/S4, M4-C3-S1/S2/S3/S4). **Latency Trap: Required Callout** (final reminder: workstation/cloud training, Jetson inference—never cloud real-time control). Artifacts: Assessment Alignment Matrix table, capstone end-to-end test report, 84-section completion checklist, GitHub Pages deployment verification.

**Deployment Tasks**:
1. Final validation scripts:
   - `scripts/validate-structure.sh`: Verify 84 files at `docs/M[1-4]/C[1-3]/S[1-7].md` ✅
   - `scripts/validate-latency-traps.sh`: Count 36 placements (22 Required + 14 Mention) ✅
   - `scripts/validate-links.sh`: Check for broken internal links ✅
2. Docusaurus build: `npm run build` (no errors/warnings) ✅
3. GitHub Actions workflow: Push to `main` branch → triggers deploy.yml → builds site → deploys to `gh-pages` branch ✅
4. Verify site live at `https://<username>.github.io/physical-ai-textbook/` ✅
5. Test navigation: 4 modules × 3 chapters × 7 sections visible in sidebar ✅
6. Test search: Search for "Jetson Orin", "Latency Trap", "Capstone" returns relevant results ✅
7. Test mobile: Site responsive on mobile devices ✅
8. Update README.md: Add setup instructions (hardware requirements, software install, build commands) ✅

**Quality Checks**:
- ✅ Structure count: 1 file written (cumulative: 84/84) **ALL SECTIONS COMPLETE**
- ✅ Latency trap coverage: M4-C3-S7 (Required: 14/22 completed; Mention: All 14 sections link to M3-C3-S6)
- ✅ Assessment Alignment Matrix: All 6 assessments mapped to section IDs (no gaps)
- ✅ GitHub Pages deployment: Site live, all sections accessible, sidebar navigation functional

**Capstone Progression**: **CAPSTONE COMPLETE**. Full Voice → Plan → Navigate → Manipulate → Sim-to-Real pipeline validated in simulation (M4-C3-S4). Optional physical deployment documented (M4-C3-S5).

**Module 4 Completion Gate**: M4-C3-S7 checklist must pass (capstone end-to-end test succeeds, Assessment Alignment Matrix complete). Freeze M4 content.

**Assessment 6 Preparation (Complete)**: M4-C1-S1/S2/S3, M4-C2-S1/S2/S4, M4-C3-S1/S2/S3/S4 fully prepare VLA Integration & Capstone Demo assessment.

**Milestone 5 Complete**: GitHub Pages deployed. Textbook live.

---

## 3) Writing Workflow (Fast + Error-Free)

### Step 1: Generate Sections
- Use specification (spec.md Section B: 84-Section Map) as source of truth
- For each section Mx-Cy-Sz:
  1. Extract from spec: Title, Purpose, Brief Anchors, Capstone Link, Required Artifacts, Latency Trap placement, Hardware mention
  2. Write Markdown file with structure:
     ```markdown
     # [Title]

     ## Overview
     [Purpose paragraph from spec]

     ## Hardware Requirements
     [List required hardware: Workstation/Edge Kit/Cloud/Robot Lab]

     ## Connection to Capstone
     [Explicit link to Voice/Plan/Actions/Navigate/Obstacles/Vision/Manipulation]

     ## Implementation
     [Technical content with code examples]

     ### [Artifact 1 Title]
     ```[language]
     [Code or configuration]
     ```

     ### [Artifact 2 Title]
     ...

     ## Latency Trap Warning (if applicable)
     [Standard callout or abbreviated mention from Clarification Report spec.md Section F]

     ## Next Steps
     [Link to next section or chapter]
     ```
  3. Save to `docs/M[x]/C[y]/S[z].md`

### Step 2: Structural Compliance Check
- Run `scripts/validate-structure.sh`:
  - Verify 84 files exist at correct paths
  - Check directory hierarchy matches 4×3×7
  - Validate file naming (S1.md, not s1.md or section1.md)
- Expected output: "✅ All 84 sections found at correct paths"

### Step 3: Cross-Link Validation
- **URDF Cross-Link** (after M1-C3-S1, M2-C1-S1, M3-C1-S3 written):
  - Verify M2-C1-S1 references `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf` from M1-C3-S1
  - Verify M3-C1-S3 references same URDF path (no duplication)
- **Sim-to-Real Cross-Link** (after M2-C2-S7, M3-C1-S5, M3-C2-S3, M3-C3-S5/S6 written):
  - Verify M3-C2-S3 references M2-C2-S7 (Gazebo data) + M3-C1-S5 (Isaac data) for training
  - Verify M3-C3-S6 documents SCP/USB weight flashing (PRIMARY REFERENCE)
  - Verify all 14 "Mention" sections link to M3-C3-S6
- **VLA→Actions Cross-Link** (after M4-C1-S2/S3, M4-C2-S4, M1-C2-S2 written):
  - Verify M4-C2-S4 references M1-C2-S2 (ROS 2 Actions)
  - Verify mapping table exists (e.g., "navigate to X" → `NavigateToPose` action)
- Run `scripts/validate-links.sh` to catch broken internal links

### Step 4: Module Consistency Check Sections (Mx-C3-S7)
- **After each module complete** (M1-C3-S7, M2-C3-S7, M3-C3-S7, M4-C3-S7):
  1. Create checklist table:
     ```markdown
     | Check | Status | Evidence |
     |-------|--------|----------|
     | Sim-to-Real Adherence | ✅/❌ | [Brief verification] |
     | Capstone Readiness | ✅/❌ | [Which capstone steps enabled] |
     | Hardware Compliance | ✅/❌ | [Runnable on Economy Kit/Workstation] |
     | Cross-Links Valid | ✅/❌ | [URDF/Synthetic Data/VLA→Actions] |
     | Latency Traps Present | ✅/❌ | [Count Required/Mention placements] |
     ```
  2. Write test script `test-moduleX.sh` (e.g., checks ROS 2 topic list, URDF validation, IMU data streaming for M1)
  3. Include screenshot/video evidence (e.g., RViz2 showing URDF with TF frames for M1-C3-S7)
- **Freeze module content** after checklist passes; no further edits unless critical bugs

### Step 5: Build Docusaurus Locally
- Run `npm run build` after every 7 sections (weekly)
- Check for build errors:
  - Broken Markdown links → Fix with correct relative paths
  - Invalid MDX syntax → Correct JSX components
  - Missing images → Add to `static/img/`
- Expected output: "✅ Build succeeded in 2m 34s"

### Step 6: Commit
- After each weekly completion (7 sections), commit to git:
  ```bash
  git add docs/M[X]/C[Y]/*.md
  git commit -m "feat(M[X]C[Y]): Complete Chapter [Y] ([Section IDs])

  - [Brief description of chapter theme]
  - [Number] sections written with [X] code artifacts
  - Cross-links validated: [URDF/Synthetic Data/VLA]
  - Latency trap placements: [Required/Mention counts]

  Related: Assessment [N] preparation
  "
  ```
- Push to remote weekly for backup

---

## 4) Definition of Done (Measurable)

### File Existence
- ✅ **84 files exist** at required paths: `docs/M[1-4]/C[1-3]/S[1-7].md`
- Validation: `scripts/validate-structure.sh` returns "✅ All 84 sections found"

### Sidebar Structure
- ✅ **Sidebars reflect 4×3×7 exactly**: `sidebars.js` contains 4 modules → 3 chapters each → 7 sections each
- Validation: Manual inspection of sidebar navigation in built site

### Scope Lock Compliance
- ✅ **Content obeys scope lock** (Constitution Article II): Only brief-approved tools (ROS 2 Humble, Gazebo Harmonic, Unity, Isaac Sim/Lab/ROS, RealSense D435i/D455, Whisper, Jetson Orin, Unitree G1/Go2, Robotis OP3, AWS g5.2xlarge)
- Validation: Manual review of all 84 sections; no external dependencies beyond brief

### Latency Trap Enforcement
- ✅ **36 placements confirmed**: 22 "Required Callout" + 14 "Mention"
- Validation: `scripts/validate-latency-traps.sh` counts placements; expected output "✅ 22 Required, 14 Mention (36 total)"

### Information Density
- ✅ **Every section includes ≥1 runnable code artifact** or technical procedure (Constitution Article X)
- Validation: Manual review of "Required Artifacts" field in each section

### Capstone Completeness
- ✅ **M4-C3 provides full Voice → Plan → Navigate → Manipulate → Sim-to-Real pipeline**:
  - M4-C3-S1: Voice→Plan integration functional
  - M4-C3-S2: Plan→Navigate integration functional (uses M3-C2-S5 Nav2)
  - M4-C3-S3: Navigate→Manipulate integration functional (uses M3-C2-S3 object detection)
  - M4-C3-S4: Capstone fully achievable in Gazebo + Isaac Sim (simulation-first rule)
  - M4-C3-S5: Physical deployment documented as optional (Sim-to-Real extension)
- Validation: M4-C3-S7 end-to-end test report

### Assessment Alignment
- ✅ **All 6 assessments fully preparable**:
  - Assessment 1 (ROS 2 Quiz): M1-C1-S4/S5/S6/S7, M1-C2-S1/S3/S4
  - Assessment 2 (URDF & Simulation Lab): M1-C3-S1/S2/S3/S4, M2-C1-S1/S3/S7
  - Assessment 3 (Sensor Simulation): M2-C2-S1 through S7
  - Assessment 4 (Isaac Sim & RL): M3-C1-S1/S2/S3/S5, M3-C3-S2/S3/S4
  - Assessment 5 (Edge Deployment & Nav2): M1-C1-S2, M3-C2-S1/S2/S3/S5, M3-C3-S6
  - Assessment 6 (VLA & Capstone): M4-C1-S1/S2/S3, M4-C2-S1/S2/S4, M4-C3-S1/S2/S3/S4
- Validation: M4-C3-S7 Assessment Alignment Matrix table

### Site Build & Deployment
- ✅ **Site builds without errors**: `npm run build` succeeds
- ✅ **Ready for GitHub Pages deployment**: GitHub Actions workflow configured, site deploys to `gh-pages` branch
- ✅ **Site accessible**: `https://<username>.github.io/physical-ai-textbook/` live
- Validation: Manual site check (navigation, search, mobile responsiveness)

### Cross-Link Validation
- ✅ **URDF cross-links valid**: M1-C3-S1 (define) → M2-C1-S1 (Gazebo spawn) → M3-C1-S3 (USD conversion)
- ✅ **Sim-to-Real cross-links valid**: M2-C2-S7 + M3-C1-S5 (synthetic data) → M3-C2-S3 (training) → M3-C3-S5 (ONNX) → M3-C3-S6 (flash to Jetson)
- ✅ **VLA→Actions cross-links valid**: M4-C1-S2 (Whisper) → M4-C1-S3 (parsing) → M4-C2-S4 (grounding) → M1-C2-S2 (Actions)
- Validation: `scripts/validate-links.sh` passes (no broken internal links)

### Module Consistency Checks Passed
- ✅ **M1-C3-S7**: ROS 2 functional, URDF valid, IMU streaming
- ✅ **M2-C3-S7**: Gazebo physics stable, Unity-ROS bridge operational, sensor accuracy validated
- ✅ **M3-C3-S7**: Isaac Sim operational, VSLAM accurate, RL policy transferred to Jetson
- ✅ **M4-C3-S7**: Capstone pipeline functional, Assessment Alignment Matrix complete
- Validation: Checklist tables in each Mx-C3-S7 section show all ✅

---

**END OF 13-WEEK EXECUTION ROADMAP**
