# Tasks: Physical AI & Humanoid Robotics Textbook (84-Section Implementation)

**Input**: Design documents from `/specs/001-textbook-spec/`
**Prerequisites**: plan.md (13-week roadmap), spec.md (84-section map, hardware tables, assessment alignment)

**Organization**: Tasks are grouped by implementation phase matching the 13-week roadmap. Each phase maps to weekly deliverables from plan.md.

---

## Format: `[TaskID] [P?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

---

## A) Repository & Docusaurus Setup Tasks (Week 1)

### Phase 1: Docusaurus Initialization

- [ ] T001 Initialize Docusaurus project in repository root using `npx create-docusaurus@latest physical-ai-textbook classic --typescript`
- [ ] T002 Configure `package.json` with project metadata (name: "physical-ai-humanoid-robotics-textbook", description, repository URL)
- [ ] T003 Configure `docusaurus.config.js` with site metadata: title "Physical AI & Humanoid Robotics", tagline "Bridge the Digital Brain and the Physical Body", organizationName, projectName, baseUrl for GitHub Pages
- [ ] T004 Configure docs plugin in `docusaurus.config.js`: routeBasePath "/"  (docs-only mode), sidebarPath "./sidebars.js"
- [ ] T005 [P] Install additional dependencies: `npm install --save @docusaurus/theme-mermaid markdown-link-check`
- [ ] T006 [P] Create `.gitignore` file excluding `node_modules/`, `.docusaurus/`, `build/`

---

## B) Content Scaffolding Tasks (Week 1)

### Phase 2: Directory Structure & File Creation

- [ ] T007 Create Module 1 directory structure: `mkdir -p docs/M1/C1 docs/M1/C2 docs/M1/C3`
- [ ] T008 [P] Create Module 2 directory structure: `mkdir -p docs/M2/C1 docs/M2/C2 docs/M2/C3`
- [ ] T009 [P] Create Module 3 directory structure: `mkdir -p docs/M3/C1 docs/M3/C2 docs/M3/C3`
- [ ] T010 [P] Create Module 4 directory structure: `mkdir -p docs/M4/C1 docs/M4/C2 docs/M4/C3`
- [ ] T011 Create static assets directories: `mkdir -p static/img static/code static/datasets`
- [ ] T012 [P] Create source directories: `mkdir -p src/components src/css src/snippets`
- [ ] T013 [P] Create scripts directory: `mkdir -p scripts`
- [ ] T014 [P] Create GitHub Actions directory: `mkdir -p .github/workflows`

### Phase 3: Sidebar Configuration

- [ ] T015 Write `sidebars.js` with 4-level hierarchy (4 modules × 3 chapters × 7 sections) reflecting exact 4×3×7 structure from spec.md
- [ ] T016 Configure sidebar to use document IDs matching file paths (e.g., 'M1/C1/S1', 'M1/C1/S2', ... 'M4/C3/S7')

### Phase 4: Placeholder File Generation

- [ ] T017 Create 84 placeholder Markdown files with Docusaurus frontmatter using bash loop: `for m in {1..4}; do for c in {1..3}; do for s in {1..7}; do touch docs/M$m/C$c/S$s.md; done; done; done`
- [ ] T018 Populate frontmatter for all 84 files with title (from spec.md Section B), sidebar_position (S1=1, S2=2, ... S7=7), keywords (ROS 2, Gazebo, Isaac, VLA as appropriate)

### Phase 5: Template Skeleton Insertion

- [ ] T019 Create standard section template in `src/templates/section-template.md` with structure: Overview, Hardware Requirements, Connection to Capstone, Implementation, Latency Trap Warning (placeholder), Next Steps
- [ ] T020 Insert template skeleton into all 84 Markdown files with [TODO] placeholders for content areas

---

## C) Module 1 Writing Tasks (Weeks 2-3: 21 Sections)

### Week 2: Module 1 - Chapter 1 (Foundations & Hardware)

- [ ] T021 [P] Write M1-C1-S1 (Workstation Setup) in `docs/M1/C1/S1.md`: Ubuntu 22.04 install guide, NVIDIA driver setup, CUDA verification. Artifacts: `setup-workstation.sh`, `test-gpu.sh`. Latency Trap: Mention. Hardware: Workstation. Done when: Section includes bash script + verification commands + hardware requirements table.

- [ ] T022 [P] Write M1-C1-S2 (Jetson Edge Kit) in `docs/M1/C1/S2.md`: JetPack 6.x flashing, RealSense/IMU/ReSpeaker connection, peripheral validation. Artifacts: `flash-jetson.sh`, `test-peripherals.sh`. Latency Trap: Required Callout. Hardware: Edge Kit (Economy $700). Done when: Section includes flashing procedure + peripheral tests + Economy Kit hardware table + latency callout block.

- [ ] T023 [P] Write M1-C1-S3 (Physical AI Principles) in `docs/M1/C1/S3.md`: Embodied vs. disembodied AI comparison, physics constraints. Artifacts: Comparison table, `physics-aware-planning.py` pseudocode, humanoid diagram. Latency Trap: None. Hardware: Simulation-only. Done when: Section includes comparison table + Python pseudocode + annotated SVG diagram.

- [ ] T024 Write M1-C1-S4 (ROS 2 Installation) in `docs/M1/C1/S4.md`: ROS 2 Humble apt install, colcon workspace setup, `.bashrc` sourcing. Artifacts: `install-ros2.sh`, `~/ros2_ws` directory tree. Latency Trap: None. Hardware: Workstation. Done when: Section includes install script + workspace structure + sourcing instructions. Assessment 1 preparation.

- [ ] T025 Write M1-C1-S5 (Nodes/Topics/Pub-Sub) in `docs/M1/C1/S5.md`: ROS 2 graph explanation, publisher/subscriber implementation. Artifacts: `joint_cmd_publisher.py`, `sensor_listener.py`, `rqt_graph` screenshot. Latency Trap: None. Hardware: Workstation. Done when: Section includes Python publisher + subscriber + graph visualization. Assessment 1 preparation.

- [ ] T026 Write M1-C1-S6 (TF2 Fundamentals) in `docs/M1/C1/S6.md`: Coordinate frames, static transform broadcaster, RViz2 visualization. Artifacts: `broadcast_static_tf.py`, `rviz2_config.rviz`, TF tree diagram. Latency Trap: None. Hardware: Workstation. Done when: Section includes transform broadcaster + RViz2 config + TF tree visualization. Assessment 1 preparation.

- [ ] T027 Write M1-C1-S7 (CLI Tools) in `docs/M1/C1/S7.md`: `ros2 topic`, `ros2 node`, `ros2 bag`, `ros2 doctor` usage. Artifacts: `ros2-cli-cheatsheet.md`, `record-rosbag.sh`. Latency Trap: None. Hardware: Workstation. Done when: Section includes CLI cheat sheet + rosbag commands + example outputs. Assessment 1 preparation.

### Week 3: Module 1 - Chapters 2 & 3 (ROS 2 Logic + URDF/TF2)

- [ ] T028 [P] Write M1-C2-S1 (Services) in `docs/M1/C2/S1.md`: ROS 2 service server/client, IK service example. Artifacts: `ik_service.py`, `ik_client.py`, `.srv` definition. Latency Trap: Mention (avoid cloud services in real-time loops). Hardware: Workstation. Done when: Section includes service server + client + custom `.srv` file. Assessment 1 preparation.

- [ ] T029 [P] Write M1-C2-S2 (Actions) in `docs/M1/C2/S2.md`: Long-running tasks, action server/client, NavigateToPose example. Artifacts: `navigate_action_server.py`, `.action` definition, feedback callback. Latency Trap: None. Hardware: Workstation. Cross-link: M4-C2-S4 (language→action grounding) triggers actions defined here. Done when: Section includes action server + client + `.action` definition + feedback examples.

- [ ] T030 [P] Write M1-C2-S3 (Parameters) in `docs/M1/C2/S3.md`: Runtime parameter management, YAML configs. Artifacts: `param_node.py`, `config.yaml`, `ros2 param` commands. Latency Trap: None. Hardware: Workstation. Done when: Section includes parameter node + YAML config + CLI commands. Assessment 1 preparation.

- [ ] T031 [P] Write M1-C2-S4 (Launch Files) in `docs/M1/C2/S4.md`: Multi-node orchestration, Python launch files. Artifacts: `capstone_bringup.launch.py`, launch file tutorial. Latency Trap: None. Hardware: Workstation. Done when: Section includes Python launch file + node configuration + remapping examples. Assessment 1 preparation.

- [ ] T032 [P] Write M1-C2-S5 (Custom Messages) in `docs/M1/C2/S5.md`: `.msg`/`.srv`/`.action` definitions, CMakeLists.txt rules. Artifacts: `HumanoidJointState.msg`, build instructions. Latency Trap: None. Hardware: Workstation. Done when: Section includes `.msg` file + CMakeLists.txt rules + build/source instructions.

- [ ] T033 [P] Write M1-C2-S6 (QoS Policies) in `docs/M1/C2/S6.md`: Reliability vs. best-effort QoS for sensors/commands. Artifacts: `qos_profiles.py`, QoS comparison table. Latency Trap: Mention (QoS affects latency). Hardware: Workstation. Done when: Section includes QoS profile definitions + comparison table + `rqt_graph` examples.

- [ ] T034 [P] Write M1-C2-S7 (Rosbag) in `docs/M1/C2/S7.md`: Recording/playback for debugging. Artifacts: `ros2 bag record` commands, compression options. Latency Trap: None. Hardware: Workstation. Done when: Section includes record/playback commands + rosbag info examples + compression guide.

- [ ] T035 [P] Write M1-C3-S1 (URDF Basics) in `docs/M1/C3/S1.md`: **CANONICAL URDF DEFINITION** at `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf`. Links, joints, kinematic chains. Artifacts: `humanoid_arm.urdf`, XML tag explanations, RViz2 visualization. Latency Trap: None. Hardware: Workstation. Cross-link: M2-C1-S1 (Gazebo spawn) and M3-C1-S3 (USD conversion) reference this URDF. Done when: Section includes URDF file + XML explanations + RViz2 screenshot + canonical path documented. Assessment 2 preparation.

- [ ] T036 [P] Write M1-C3-S2 (Xacro) in `docs/M1/C3/S2.md`: Xacro macros for modular URDFs. Artifacts: `humanoid_torso.xacro`, `xacro` CLI conversion. Latency Trap: None. Hardware: Workstation. Done when: Section includes Xacro file + macro definitions + conversion command + parameterized joint limits. Assessment 2 preparation.

- [ ] T037 [P] Write M1-C3-S3 (Forward Kinematics) in `docs/M1/C3/S3.md`: FK computation, `robot_state_publisher`. Artifacts: `joint_state_publisher.py`, `robot_state_publisher` launch config, RViz2 TF visualization, FK script. Latency Trap: None. Hardware: Workstation. Done when: Section includes FK calculation + joint state publisher + RViz2 visualization. Assessment 2 preparation.

- [ ] T038 [P] Write M1-C3-S4 (Inverse Kinematics) in `docs/M1/C3/S4.md`: IK solver (KDL/TracIK), ROS 2 service wrapper. Artifacts: `ik_solver.py`, KDL/TracIK integration. Latency Trap: Mention (IK on workstation for planning). Hardware: Workstation. Done when: Section includes IK solver + service wrapper + solution validation. Assessment 2 preparation.

- [ ] T039 [P] Write M1-C3-S5 (IMU Integration) in `docs/M1/C3/S5.md`: USB IMU connection to Jetson, `robot_localization` EKF. Artifacts: `imu_driver.py`, `sensor_msgs/Imu` publisher, EKF config YAML, RViz2 odometry visualization. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes IMU driver + EKF config + odometry visualization.

- [ ] T040 [P] Write M1-C3-S6 (Collision Geometries) in `docs/M1/C3/S6.md`: Collision meshes, MoveIt2 collision checking. Artifacts: URDF with `<collision>` tags, STL meshes, MoveIt2 config, collision checking script. Latency Trap: None. Hardware: Workstation. Done when: Section includes collision URDF + STL meshes + MoveIt2 collision checking example.

- [ ] T041 Write M1-C3-S7 (Module 1 Consistency Check) in `docs/M1/C3/S7.md`: Validate ROS 2 functionality, URDF kinematics, IMU integration. Artifacts: Checklist table (Sim-to-Real Adherence, Capstone Readiness, Hardware Compliance, Cross-Links Valid, Latency Traps Present), `test-module1.sh`, RViz2 screenshot. Latency Trap: Required Callout (workstation for dev, Jetson for inference). Hardware: Workstation + Edge Kit. Done when: Section includes checklist table (all ✅) + test script + evidence screenshot + module freeze notice.

---

## C) Module 2 Writing Tasks (Weeks 4-6: 21 Sections)

### Week 4: Module 2 - Chapter 1 (Gazebo Physics)

- [ ] T042 Write M2-C1-S1 (Gazebo Installation) in `docs/M2/C1/S1.md`: Gazebo Harmonic install, spawn humanoid URDF from M1-C3-S1. Artifacts: `install-gazebo.sh`, `.world` file, launch command. Latency Trap: None. Hardware: Workstation. Cross-link: References `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf` from M1-C3-S1. Done when: Section includes Gazebo install script + world file + spawn launch command + cross-reference to M1-C3-S1 URDF. Assessment 2 preparation.

- [ ] T043 [P] Write M2-C1-S2 (Physics Engines) in `docs/M2/C1/S2.md`: ODE/Bullet/DART comparison, solver tuning. Artifacts: SDF physics config, timestep tuning guide, stability test. Latency Trap: None. Hardware: Workstation. Done when: Section includes engine comparison table + SDF config + solver tuning guide + stability test script.

- [ ] T044 [P] Write M2-C1-S3 (World Building) in `docs/M2/C1/S3.md`: Terrain (stairs, ramps), obstacles (walls, boxes), props (blocks). Artifacts: SDF world file, Blender mesh export guide, `gz model` CLI usage. Latency Trap: None. Hardware: Workstation. Done when: Section includes SDF world file + Blender export guide + gz model CLI examples. Assessment 2 preparation.

- [ ] T045 [P] Write M2-C1-S4 (Collision Detection) in `docs/M2/C1/S4.md`: Contact surface parameters (friction, bounce), contact sensors. Artifacts: SDF collision config, contact force test script. Latency Trap: None. Hardware: Workstation. Done when: Section includes SDF collision parameters + contact sensor plugin + force test script.

- [ ] T046 [P] Write M2-C1-S5 (Friction Models) in `docs/M2/C1/S5.md`: Static/dynamic/rolling friction, foot-ground interaction. Artifacts: SDF friction params, Coulomb friction explanation, slip detection script. Latency Trap: None. Hardware: Workstation. Done when: Section includes friction parameters + Coulomb model explanation + slip detection example.

- [ ] T047 [P] Write M2-C1-S6 (Joint Controllers) in `docs/M2/C1/S6.md`: PID controllers via `ros2_control`, actuator limits. Artifacts: `ros2_control` YAML config, PID tuning script, effort/velocity plots. Latency Trap: None. Hardware: Workstation. Done when: Section includes ros2_control YAML + PID tuning script + performance plots.

- [ ] T048 Write M2-C1-S7 (Gazebo ROS 2 Bridge) in `docs/M2/C1/S7.md`: `ros_gz_bridge` topic mapping. Artifacts: Bridge config YAML, topic remapping examples, integrated launch file. Latency Trap: None. Hardware: Workstation. Done when: Section includes bridge config + topic remapping + integrated launch file. Assessment 2 preparation.

### Week 5: Module 2 - Chapter 2 (Sensor Simulation)

- [ ] T049 [P] Write M2-C2-S1 (RealSense D435i Simulation) in `docs/M2/C2/S1.md`: Depth camera plugin in Gazebo, point cloud publisher. Artifacts: URDF camera link with Gazebo plugin, RViz2 depth visualization. Latency Trap: None. Hardware: Workstation (simulated) + Edge Kit (real hardware). Done when: Section includes URDF camera link + Gazebo plugin config + RViz2 point cloud visualization. Assessment 3 preparation.

- [ ] T050 [P] Write M2-C2-S2 (LiDAR Simulation) in `docs/M2/C2/S2.md`: 2D/3D LiDAR, PCL filtering. Artifacts: URDF LiDAR link, ray sensor plugin, `pcl_filter.py`. Latency Trap: None. Hardware: Workstation. Done when: Section includes URDF LiDAR + Gazebo ray sensor + PCL filtering script. Assessment 3 preparation.

- [ ] T051 [P] Write M2-C2-S3 (Depth Calibration) in `docs/M2/C2/S3.md`: Camera intrinsics, `camera_calibration` procedure. Artifacts: `camera_info` YAML, calibration checkerboard guide. Latency Trap: None. Hardware: Workstation (sim) + Edge Kit (real calibration). Done when: Section includes camera_info YAML + calibration procedure + intrinsics explanation. Assessment 3 preparation.

- [ ] T052 [P] Write M2-C2-S4 (IMU Noise Models) in `docs/M2/C2/S4.md`: Gaussian noise, bias drift in Gazebo IMU plugin, sensor fusion. Artifacts: Gazebo IMU noise config, `robot_localization` EKF tuning. Latency Trap: None. Hardware: Workstation. Cross-link: Distinct from M1-C3-S5 (real hardware IMU). Done when: Section includes IMU noise config + EKF tuning + distinction from M1-C3-S5 documented. Assessment 3 preparation.

- [ ] T053 [P] Write M2-C2-S5 (RGB-D Alignment) in `docs/M2/C2/S5.md`: Align RGB+depth streams, depth hole filling. Artifacts: `align_rgbd.py`, depth accuracy validation plot. Latency Trap: None. Hardware: Workstation (sim) + Edge Kit (real validation). Done when: Section includes RGB-D alignment script + depth hole filling algorithm + accuracy plot. Assessment 3 preparation.

- [ ] T054 [P] Write M2-C2-S6 (Simulating Latency) in `docs/M2/C2/S6.md`: Artificial latency/packet loss for robustness testing. Artifacts: `delay_node.py`, packet drop script, latency histogram. Latency Trap: Mention (simulation zero-latency vs. real edge latency). Hardware: Workstation (sim latency) + Edge Kit (real latency). Done when: Section includes delay node + packet drop script + latency histogram + latency trap mention. Assessment 3 preparation.

- [ ] T055 Write M2-C2-S7 (Synthetic Data) in `docs/M2/C2/S7.md`: Capture RGB-D images/labels for object detection training. Artifacts: `capture_synthetic_data.py`, COCO-format annotations, dataset directory structure. Latency Trap: Mention (synthetic training on workstation). Hardware: Workstation (cloud for large-scale). Cross-link: Feeds M3-C2-S3 (object detection training). Done when: Section includes data capture script + COCO annotations + dataset structure + cross-reference to M3-C2-S3. Assessment 3 preparation.

### Week 6: Module 2 - Chapter 3 (Unity & HRI)

- [ ] T056 [P] Write M2-C3-S1 (Unity Installation) in `docs/M2/C3/S1.md`: Unity Editor, Unity Robotics Hub ROS 2 package, ROS TCP Connector. Artifacts: Unity project setup guide, ROS 2 pub/sub demo. Latency Trap: None. Hardware: Workstation. Done when: Section includes Unity install guide + Robotics Hub setup + ROS TCP Connector config + pub/sub demo.

- [ ] T057 [P] Write M2-C3-S2 (High-Fidelity Rendering) in `docs/M2/C3/S2.md`: HDRP, global illumination, high-res textures. Artifacts: Unity HDRP scene setup, material library, Unity vs. Gazebo comparison. Latency Trap: None. Hardware: Workstation. Done when: Section includes HDRP setup guide + material library + rendered image comparison (Unity vs. Gazebo).

- [ ] T058 [P] Write M2-C3-S3 (Unity-ROS Bridge) in `docs/M2/C3/S3.md`: Stream Unity camera/IMU to ROS 2 topics. Artifacts: Unity sensor publisher scripts, ROS 2 subscriber verification, latency measurement. Latency Trap: None. Hardware: Workstation. Done when: Section includes Unity publisher scripts + ROS 2 subscriber test + latency measurement.

- [ ] T059 [P] Write M2-C3-S4 (Humanoid Avatar) in `docs/M2/C3/S4.md`: Import humanoid model, rig with Unity Avatar, play `JointState`-driven animations. Artifacts: Unity humanoid prefab, Avatar mapping config, animation script. Latency Trap: None. Hardware: Workstation. Done when: Section includes Unity prefab + Avatar mapping + animation script + playback demo.

- [ ] T060 [P] Write M2-C3-S5 (Voice Command UI) in `docs/M2/C3/S5.md`: Unity UI canvas for voice input, display task progress, connect to ROS 2 actions. Artifacts: Unity UI prefab, voice input handler, action client integration. Latency Trap: None. Hardware: Workstation. Done when: Section includes Unity UI prefab + voice input handler + ROS 2 action client integration.

- [ ] T061 [P] Write M2-C3-S6 (Domain Randomization) in `docs/M2/C3/S6.md`: Randomize textures/lighting/object placements for sim-to-real transfer. Artifacts: Unity randomization script, randomized dataset export, perception accuracy comparison. Latency Trap: None. Hardware: Workstation. Done when: Section includes randomization script + dataset export + before/after perception accuracy comparison.

- [ ] T062 Write M2-C3-S7 (Module 2 Consistency Check) in `docs/M2/C3/S7.md`: Validate Gazebo physics, Unity-ROS bridge functionality, sensor accuracy. Artifacts: Checklist table, `test-module2.sh`, Gazebo + Unity screenshots. Latency Trap: Required Callout (simulation zero-latency vs. edge). Hardware: Workstation. Done when: Section includes checklist table (all ✅) + test script + Gazebo/Unity evidence + module freeze notice.

---

## D) Module 3 Writing Tasks (Weeks 7-9: 21 Sections)

### Week 7: Module 3 - Chapter 1 Part 1 (Omniverse & USD)

- [ ] T063 [P] Write M3-C1-S1 (Omniverse Installation) in `docs/M3/C1/S1.md`: Omniverse Launcher, Nucleus (local/cloud), Isaac Sim install, GPU verification. Artifacts: `install-omniverse.sh`, sample scene launch, GPU benchmark. Latency Trap: Mention (Isaac Sim on workstation/cloud for training). Hardware: Workstation (cloud for heavy rendering). Done when: Section includes Omniverse install script + Nucleus config + sample scene + GPU benchmark. Assessment 4 preparation.

- [ ] T064 [P] Write M3-C1-S2 (USD Fundamentals) in `docs/M3/C1/S2.md`: USD prims/attributes/relationships, Python USD reader. Artifacts: USD reader script, sample USD file, prim hierarchy diagram. Latency Trap: None. Hardware: Workstation. Done when: Section includes Python USD reader + sample USD file + prim hierarchy diagram. Assessment 4 preparation.

- [ ] T065 Write M3-C1-S3 (URDF→USD Conversion) in `docs/M3/C1/S3.md`: Convert M1-C3-S1 humanoid URDF to USD for Isaac Sim. Artifacts: `urdf_to_usd.py`, Isaac Sim USD asset, physics test comparison. Latency Trap: None. Hardware: Workstation. Cross-link: References `~/ros2_ws/src/humanoid_description/urdf/humanoid.urdf` from M1-C3-S1. Done when: Section includes URDF→USD conversion script + USD asset + physics comparison (Gazebo vs. Isaac) + cross-reference to M1-C3-S1. Assessment 4 preparation.

- [ ] T066 [P] Write M3-C1-S4 (Isaac Physics) in `docs/M3/C1/S4.md`: PhysX GPU physics, solver tuning for humanoid contact. Artifacts: Isaac Sim physics config, solver tuning table, GPU vs. CPU speedup benchmark. Latency Trap: None. Hardware: Workstation. Done when: Section includes PhysX config + solver tuning table + GPU speedup benchmark.

- [ ] T067 Write M3-C1-S5 (Synthetic Data Replicator) in `docs/M3/C1/S5.md`: Isaac Sim Replicator API for large-scale randomized datasets. Artifacts: Replicator Python script, randomized scene config, COCO dataset output. Latency Trap: Mention (training on workstation/cloud). Hardware: Workstation (cloud for large-scale). Cross-link: Complements M2-C2-S7 (Gazebo data); feeds M3-C2-S3 (object detection training). Done when: Section includes Replicator script + scene randomization + COCO dataset + cross-references to M2-C2-S7 and M3-C2-S3. Assessment 4 preparation.

- [ ] T068 [P] Write M3-C1-S6 (OmniGraph ROS 2) in `docs/M3/C1/S6.md`: Connect Isaac Sim to ROS 2 via OmniGraph nodes (pub/sub/actions). Artifacts: OmniGraph ROS 2 setup, action client integration, topic latency measurement. Latency Trap: None. Hardware: Workstation. Done when: Section includes OmniGraph setup guide + ROS 2 integration examples + latency measurement.

### Week 8: Module 3 - Chapter 1 Part 2 + Chapter 2 (Isaac ROS)

- [ ] T069 Write M3-C1-S7 (Cloud Rendering) in `docs/M3/C1/S7.md`: Omniverse Farm (cloud), Nucleus cloud storage, batch rendering. **Ether Lab: $205/quarter breakdown ($1.20/hr × ~170 GPU-hrs)**. Artifacts: Farm submission script, Nucleus cloud config, cost estimation table. Latency Trap: Required Callout (cloud for training, NOT real-time control). Hardware: Cloud (AWS g5.2xlarge). Done when: Section includes Farm submission script + Nucleus cloud config + $205/quarter cost breakdown table + latency callout + use cases (rendering, RL, synthetic data).

- [ ] T070 [P] Write M3-C2-S1 (Isaac ROS Installation) in `docs/M3/C2/S1.md`: Install Isaac ROS GEMs (accelerated perception) on workstation + Jetson. Artifacts: Isaac ROS install script, GEM verification commands, GPU vs. CPU benchmark. Latency Trap: None. Hardware: Workstation + Edge Kit. Done when: Section includes Isaac ROS install script + GEM verification + performance benchmark. Assessment 5 preparation.

- [ ] T071 Write M3-C2-S2 (cuVSLAM) in `docs/M3/C2/S2.md`: NVIDIA cuVSLAM for visual-inertial odometry on Jetson. Artifacts: cuVSLAM launch file, RealSense+IMU calibration guide, VSLAM trajectory plot. Latency Trap: Required Callout (VSLAM runs on Jetson for real-time localization). Hardware: Edge Kit. Done when: Section includes cuVSLAM launch file + calibration guide + trajectory plot + latency callout. Assessment 5 preparation.

- [ ] T072 Write M3-C2-S3 (Object Detection) in `docs/M3/C2/S3.md`: TensorRT-optimized detection (YOLO/DetectNet) on Jetson. Artifacts: TensorRT conversion script, Isaac ROS DNN launch file, latency benchmark. Latency Trap: Required Callout (training on workstation/cloud → TensorRT inference on Jetson). Hardware: Edge Kit (training on Workstation/Cloud). Cross-link: Uses synthetic data from M2-C2-S7 + M3-C1-S5 for training. Done when: Section includes TensorRT conversion + Isaac ROS DNN launch + latency benchmark + cross-references to M2-C2-S7/M3-C1-S5 + latency callout. Assessment 5 preparation.

- [ ] T073 [P] Write M3-C2-S4 (Depth Segmentation) in `docs/M3/C2/S4.md`: RANSAC plane fitting for floor/object segmentation. Artifacts: `plane_fit_ransac.py`, segmented point cloud publisher, object centroid extraction. Latency Trap: None. Hardware: Workstation + Edge Kit. Done when: Section includes RANSAC plane fitting script + segmented cloud publisher + centroid extraction.

- [ ] T074 Write M3-C2-S5 (Nav2 Integration) in `docs/M3/C2/S5.md`: Configure ROS 2 Nav2 for humanoid navigation (costmaps, DWB planner, recovery behaviors). Artifacts: Nav2 params YAML, costmap config, behavior tree XML, Gazebo navigation test. Latency Trap: None. Hardware: Workstation (sim) + Edge Kit (deployment). Cross-link: M4-C3-S2 uses Nav2 from this section. Done when: Section includes Nav2 params + costmap config + behavior tree + navigation test + cross-reference note for M4-C3-S2. Assessment 5 preparation.

- [ ] T075 [P] Write M3-C2-S6 (Occupancy Mapping) in `docs/M3/C2/S6.md`: Build 2D occupancy grids with `slam_toolbox` for Nav2 localization. Artifacts: `slam_toolbox` config, map building script, saved map (PGM+YAML). Latency Trap: None. Hardware: Workstation + Edge Kit. Done when: Section includes slam_toolbox config + map building script + saved map files.

- [ ] T076 [P] Write M3-C2-S7 (Semantic Segmentation) in `docs/M3/C2/S7.md`: TensorRT semantic segmentation (ESANet/SegFormer) on Jetson for scene understanding. Artifacts: TensorRT model, Isaac ROS integration, segmented image visualization, latency benchmark. Latency Trap: None. Hardware: Edge Kit (training on Workstation/Cloud). Done when: Section includes TensorRT semantic model + Isaac ROS integration + segmented visualization + latency benchmark.

### Week 9: Module 3 - Chapter 3 (Sim-to-Real/RL)

- [ ] T077 [P] Write M3-C3-S1 (Bipedal Locomotion) in `docs/M3/C3/S1.md`: ZMP, COG, gait phases, open-loop walking controller. Artifacts: ZMP calculation script, gait trajectory planner, Gazebo walking demo, stability analysis plot. Latency Trap: None. Hardware: Workstation (simulation). Done when: Section includes ZMP calculation + gait planner + Gazebo demo + stability plot.

- [ ] T078 Write M3-C3-S2 (Isaac Gym Setup) in `docs/M3/C3/S2.md`: Install Isaac Gym, create humanoid RL environment (state/action spaces), configure parallel simulation. **Ether Lab: Cloud option for large-scale training**. Artifacts: Isaac Gym install script, custom environment code, state/action definitions, parallel env verification. Latency Trap: Required Callout (RL training on workstation/cloud). Hardware: Workstation (cloud for large-scale). Done when: Section includes Isaac Gym install + custom environment + state/action definitions + parallel env verification + Ether Lab cloud option documented + latency callout. Assessment 4 preparation.

- [ ] T079 Write M3-C3-S3 (PPO Training) in `docs/M3/C3/S3.md`: Train bipedal walking policy with PPO, monitor rewards, hyperparameter tuning. **Ether Lab: Cloud training option**. Artifacts: PPO training script, reward function code, training curves, tensorboard logs, hyperparameter guide. Latency Trap: Required Callout (policy training on workstation/cloud). Hardware: Workstation (cloud for faster training). Done when: Section includes PPO training script + reward function + training curves + hyperparameter guide + Ether Lab cloud option + latency callout. Assessment 4 preparation.

- [ ] T080 [P] Write M3-C3-S4 (Domain Randomization) in `docs/M3/C3/S4.md`: Randomize mass/friction/joint damping/sensor noise in Isaac Gym for sim-to-real robustness. Artifacts: Randomization config, before/after policy performance comparison, real-world transfer test. Latency Trap: None. Hardware: Workstation (training) + Robot Lab (real-world testing). Done when: Section includes randomization config + performance comparison + transfer test results. Assessment 4 preparation.

- [ ] T081 Write M3-C3-S5 (ONNX Export) in `docs/M3/C3/S5.md`: Export trained PyTorch policy to ONNX for TensorRT optimization, verify accuracy/latency on Jetson. Artifacts: `pytorch_to_onnx.py`, TensorRT conversion script, latency benchmark (Jetson Orin), accuracy validation. Latency Trap: Required Callout (ONNX model runs on Jetson for real-time inference). Hardware: Workstation (export) + Edge Kit (inference). Done when: Section includes PyTorch→ONNX export + TensorRT conversion + Jetson latency benchmark + accuracy validation + latency callout.

- [ ] T082 Write M3-C3-S6 (Flashing Weights to Jetson) in `docs/M3/C3/S6.md`: **PRIMARY LATENCY TRAP REFERENCE**. Transfer TensorRT weights to Jetson via SCP/USB, load in ROS 2 node for 50+ Hz inference. Artifacts: Model transfer script (SCP/USB), ROS 2 TensorRT inference node, latency histogram (inference time), real-time control loop validation. Latency Trap: Required Callout (includes latency comparison table: cloud 50-200ms vs. Jetson <10ms). Hardware: Edge Kit. Done when: Section includes SCP/USB transfer script + ROS 2 inference node + latency histogram + comparison table (cloud vs. Jetson) + real-time validation + PRIMARY REFERENCE designation for all Mention sections. Assessment 5 preparation.

- [ ] T083 Write M3-C3-S7 (Module 3 Consistency Check) in `docs/M3/C3/S7.md`: Validate Isaac Sim setup, VSLAM accuracy, RL policy transfer to Jetson. Artifacts: Checklist table, `test-module3.sh`, Jetson inference latency report. Latency Trap: Required Callout (critical sim-to-real transition—verify workstation training → Jetson inference). Hardware: Workstation + Edge Kit + Robot Lab. Done when: Section includes checklist table (all ✅) + test script + Jetson latency report + latency callout + module freeze notice.

---

## E) Module 4 Writing Tasks (Weeks 10-12: 21 Sections)

### Week 10: Module 4 - Chapter 1 Part 1 (Multimodal Perception)

- [ ] T084 [P] Write M4-C1-S1 (ReSpeaker Setup) in `docs/M4/C1/S1.md`: Connect ReSpeaker 4/6-mic array to Jetson, configure ALSA/PulseAudio, capture audio for VAD. Artifacts: `install-respeaker.sh`, ALSA config, `audio_capture.py`, VAD test script. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes ReSpeaker driver install + ALSA config + audio capture node + VAD test. Assessment 6 preparation.

- [ ] T085 Write M4-C1-S2 (Whisper ASR) in `docs/M4/C1/S2.md`: Deploy Whisper (tiny/base) on Jetson with TensorRT for real-time speech-to-text. Artifacts: `whisper_tensorrt.py`, ROS 2 ASR node, transcription accuracy test, latency benchmark. Latency Trap: Required Callout (Whisper inference on Jetson; avoid cloud ASR for real-time control). Hardware: Edge Kit. Cross-link: M4-C1-S3 (parsing) uses Whisper output. Done when: Section includes Whisper TensorRT conversion + ROS 2 ASR node + accuracy test + latency benchmark + latency callout + cross-reference to M4-C1-S3. Assessment 6 preparation.

- [ ] T086 Write M4-C1-S3 (Command Parsing) in `docs/M4/C1/S3.md`: Parse Whisper transcriptions into structured commands (intent/entity extraction with regex/spaCy). Artifacts: `command_parser.py`, intent/entity examples, ROS 2 command publisher, test cases. Latency Trap: None. Hardware: Edge Kit. Cross-link: Feeds M4-C2-S4 (language→action grounding). Done when: Section includes command parser + intent/entity extraction + ROS 2 publisher + test cases + cross-reference to M4-C2-S4. Assessment 6 preparation.

- [ ] T087 [P] Write M4-C1-S4 (Multimodal Fusion) in `docs/M4/C1/S4.md`: Fuse voice commands with visual context (e.g., "that block" + detected red block). Artifacts: `multimodal_fusion.py`, spatial grounding algorithm, ROS 2 fused command publisher, test cases. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes fusion node + spatial grounding algorithm + fused command publisher + pointing+voice test cases.

- [ ] T088 [P] Write M4-C1-S5 (Dialogue Management) in `docs/M4/C1/S5.md`: Dialogue manager requesting clarification when commands ambiguous. Artifacts: Dialogue state machine, clarification prompt generator, ROS 2 dialogue action server, Unity HRI feedback integration. Latency Trap: None. Hardware: Edge Kit (dialogue logic) + Workstation (Unity HRI). Done when: Section includes dialogue state machine + clarification prompts + ROS 2 action server + Unity integration.

- [ ] T089 [P] Write M4-C1-S6 (Gesture Recognition) in `docs/M4/C1/S6.md`: Detect hand gestures (pointing, waving) with MediaPipe on Jetson, combine with voice. Artifacts: MediaPipe TensorRT model, gesture detection ROS 2 node, gesture-voice fusion example, latency benchmark. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes MediaPipe TensorRT model + gesture detection node + gesture-voice fusion + latency benchmark.

### Week 11: Module 4 - Chapter 1 Part 2 + Chapter 2 (Cognitive Planning)

- [ ] T090 [P] Write M4-C1-S7 (Voice Feedback TTS) in `docs/M4/C1/S7.md`: Generate voice feedback ("Navigating to the block") with piper-TTS on Jetson. Artifacts: `tts_deployment.sh`, ROS 2 TTS node, audio playback via ReSpeaker, latency benchmark. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes TTS deployment script + ROS 2 TTS node + audio playback + latency benchmark.

- [ ] T091 Write M4-C2-S1 (LLM Integration) in `docs/M4/C2/S1.md`: Deploy Llama 3.2-3B (Jetson 8GB) or 8B (Jetson 16GB) for task decomposition ("Bring me the red block" → navigate, grasp, return). Artifacts: LLM inference script (TensorRT/ONNX), ROS 2 task planner node, decomposition examples, latency benchmark. Latency Trap: None (note: cloud LLM fallback for non-reactive planning). Hardware: Edge Kit (small LLM) or Cloud (large LLM). Done when: Section includes LLM inference script + task planner node + decomposition examples + latency benchmark + cloud fallback option. Assessment 6 preparation.

- [ ] T092 Write M4-C2-S2 (Behavior Trees) in `docs/M4/C2/S2.md`: Implement BTs with `py_trees` to execute task sequences (navigate→detect→grasp→return), handle failures. Artifacts: `behavior_tree.py`, ROS 2 action client integration, BT visualization, failure recovery examples. Latency Trap: None. Hardware: Edge Kit. Cross-link: Integrates Nav2 from M3-C2-S5. Done when: Section includes behavior tree implementation + ROS 2 action clients + BT visualization + failure recovery + cross-reference to M3-C2-S5 Nav2. Assessment 6 preparation.

- [ ] T093 [P] Write M4-C2-S3 (Context Windows) in `docs/M4/C2/S3.md`: Manage LLM context (limited tokens) by summarizing dialogue history and environment state. Artifacts: Context manager implementation, sliding window algorithm, token budget monitoring, multi-turn dialogue example. Latency Trap: None. Hardware: Edge Kit or Cloud. Done when: Section includes context manager + sliding window algorithm + token monitoring + multi-turn example.

- [ ] T094 Write M4-C2-S4 (Language→Action Grounding) in `docs/M4/C2/S4.md`: Map LLM subtasks (e.g., "navigate to object") to ROS 2 action calls (`NavigateToPose`, `GraspObject`). Artifacts: Grounding mapping table (e.g., "navigate to X" → `NavigateToPose`), ROS 2 action dispatcher, parameter extraction. Latency Trap: None. Hardware: Edge Kit. Cross-link: Triggers M1-C2-S2 (ROS 2 Actions). Done when: Section includes grounding mapping table + action dispatcher + parameter extraction + cross-reference to M1-C2-S2. Assessment 6 preparation.

- [ ] T095 [P] Write M4-C2-S5 (Error Handling) in `docs/M4/C2/S5.md`: Detect action failures (nav timeout, grasp failure), trigger LLM replanning or recovery behaviors. Artifacts: Error detection logic, replanning trigger conditions, LLM replan prompt, recovery behavior examples. Latency Trap: None. Hardware: Edge Kit or Cloud. Done when: Section includes error detection + replanning triggers + LLM replan prompts + recovery examples.

- [ ] T096 [P] Write M4-C2-S6 (Safe Action Validation) in `docs/M4/C2/S6.md`: Validate LLM-generated actions against safety constraints (collision-free paths, reachable poses). Artifacts: Safety validator node, MoveIt2 collision checking integration, validation test cases, rejection examples. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes safety validator + MoveIt2 integration + validation tests + rejection examples.

- [ ] T097 [P] Write M4-C2-S7 (Hierarchical Planning) in `docs/M4/C2/S7.md`: Implement HTN for multi-step tasks (e.g., "clean the table" → locate→grasp→place in bin). Artifacts: HTN planner implementation, hierarchical task definition, multi-step execution example. Latency Trap: None. Hardware: Edge Kit. Done when: Section includes HTN planner + hierarchical task definitions + multi-step execution example.

### Week 12: Module 4 - Chapter 3 (Capstone Integration)

- [ ] T098 Write M4-C3-S1 (Voice→Plan Integration) in `docs/M4/C3/S1.md`: Integrate Whisper ASR (M4-C1-S2) with LLM task decomposition (M4-C2-S1). Artifacts: Integrated launch file, example commands ("Bring the red block"), task plan output, ROS 2 topic flow diagram. Latency Trap: None. Hardware: Edge Kit. Cross-link: M4-C1-S2 → M4-C1-S3 → M4-C2-S1 pipeline. Done when: Section includes integrated launch file + example commands + task plan output + topic flow diagram + cross-references to M4-C1-S2/S3 and M4-C2-S1. Assessment 6 preparation.

- [ ] T099 Write M4-C3-S2 (Plan→Navigate Integration) in `docs/M4/C3/S2.md`: Connect task planner to Nav2 navigation actions. Artifacts: Behavior tree integrating planner + Nav2, navigation success criteria, obstacle avoidance test, Gazebo/Isaac Sim demo video. Latency Trap: None. Hardware: Edge Kit (real navigation) or Workstation (simulation). Cross-link: Uses M3-C2-S5 Nav2. Done when: Section includes behavior tree + Nav2 integration + success criteria + obstacle test + demo video + cross-reference to M3-C2-S5. Assessment 6 preparation.

- [ ] T100 Write M4-C3-S3 (Navigate→Manipulate Integration) in `docs/M4/C3/S3.md`: Transition from navigation to manipulation—detect object with Isaac ROS (M3-C2-S3), compute grasp pose with MoveIt2, execute grasp action. Artifacts: Object detection + grasp planning integration, MoveIt2 grasp execution, success/failure metrics, simulation + real-world demo. Latency Trap: None. Hardware: Edge Kit (real) or Workstation (sim). Cross-link: Uses M3-C2-S3 object detection. Done when: Section includes detection + grasp integration + MoveIt2 execution + success metrics + demo + cross-reference to M3-C2-S3. Assessment 6 preparation.

- [ ] T101 Write M4-C3-S4 (Capstone Simulation Testing) in `docs/M4/C3/S4.md`: Test complete Voice→Plan→Navigate→Manipulate pipeline in Gazebo + Isaac Sim. Validate diverse scenarios (varied object positions, obstacles). **Simulation-First Rule: Capstone fully achievable without physical hardware**. Artifacts: Capstone test worlds, success rate metrics, failure mode analysis, simulation video recordings. Latency Trap: None. Hardware: Workstation. Done when: Section includes capstone test worlds + success rate metrics + failure analysis + simulation videos + simulation-first rule emphasized. Assessment 6 preparation.

- [ ] T102 [P] Write M4-C3-S5 (Capstone Physical Deployment) in `docs/M4/C3/S5.md`: **OPTIONAL** Deploy to real humanoid (Unitree G1, Robotis OP3) with Jetson Orin. Artifacts: Physical deployment checklist, calibration procedures (camera, IMU), real-world test results, sim-vs-real comparison. Latency Trap: Required Callout (simulation zero-latency vs. real sensor/actuator delays 10-50ms). Hardware: Robot Lab (Unitree G1/Go2/Robotis OP3) + Edge Kit. Done when: Section includes deployment checklist + calibration procedures + real-world tests + sim-vs-real comparison + "OPTIONAL" designation + latency callout.

- [ ] T103 [P] Write M4-C3-S6 (Capstone Evaluation Rubric) in `docs/M4/C3/S6.md`: Define evaluation criteria (success rate, completion time, failure recovery, code quality). Artifacts: Grading rubric table, success rate calculation, video submission requirements, code documentation standards. Latency Trap: None. Hardware: Workstation (simulation grading) + Robot Lab (bonus points). Done when: Section includes grading rubric table + success rate calculation + submission requirements + documentation standards.

### Week 13: Module 4 - Final Consistency Check

- [ ] T104 Write M4-C3-S7 (Module 4 + Final Consistency Check) in `docs/M4/C3/S7.md`: Verify complete capstone pipeline functional (Voice → Plan → Navigate → Manipulate → Sim-to-Real). **Assessment Alignment Matrix** (map all 6 assessments to section IDs). Artifacts: Assessment Alignment Matrix table, capstone end-to-end test report, 84-section completion checklist, GitHub Pages deployment verification. Latency Trap: Required Callout (final reminder: workstation/cloud training, Jetson inference—never cloud real-time control). Hardware: All (Workstation + Edge Kit + Cloud + Robot Lab summary). Done when: Section includes Assessment Alignment Matrix (all 6 assessments mapped to section IDs) + capstone end-to-end test report + 84-section completion checklist + GitHub Pages deployment verification + latency callout + final module freeze.

---

## F) Mandatory Injection Tasks (Constitution Enforcement)

### Latency Trap Callouts

- [ ] T105 Insert "Required Callout" latency trap block in 22 sections (M1-C1-S2, M1-C3-S7, M2-C3-S7, M3-C1-S7, M3-C2-S2, M3-C2-S3, M3-C3-S2, M3-C3-S3, M3-C3-S5, M3-C3-S6, M3-C3-S7, M4-C1-S2, M4-C3-S5, M4-C3-S7) using standard callout pattern from spec.md Section F Clarification Report

- [ ] T106 Insert "Mention" latency trap reference in 14 sections (M1-C2-S1, M1-C2-S6, M1-C3-S4, M2-C2-S6, M3-C1-S5) with abbreviated text linking to M3-C3-S6 as PRIMARY REFERENCE

### Module Consistency Validation

- [ ] T107 Insert Module Consistency Validation checklist table in M1-C3-S7 (`docs/M1/C3/S7.md`) verifying: Sim-to-Real Adherence, Capstone Readiness (ROS 2 Actions, URDF kinematics), Hardware Compliance (Economy Kit compatible), Cross-Links Valid (M1-C3-S1 URDF canonical), Latency Traps Present (M1-C1-S2, M1-C2-S1/S6, M1-C3-S4/S7)

- [ ] T108 Insert Module Consistency Validation checklist table in M2-C3-S7 (`docs/M2/C3/S7.md`) verifying: Sim-to-Real Adherence (Digital Twin workflow), Capstone Readiness (sensor simulation for perception/navigation), Hardware Compliance (Gazebo/Unity on Workstation), Cross-Links Valid (M2-C1-S1→M1-C3-S1 URDF, M2-C2-S7→M3-C2-S3 synthetic data), Latency Traps Present (M2-C2-S6, M2-C3-S7)

- [ ] T109 Insert Module Consistency Validation checklist table in M3-C3-S7 (`docs/M3/C3/S7.md`) verifying: Sim-to-Real Adherence (simulation training → Jetson deployment), Capstone Readiness (cuVSLAM, object detection, Nav2, weight flashing), Hardware Compliance (Isaac Sim on Workstation/Cloud, inference on Jetson), Cross-Links Valid (M3-C1-S3→M1-C3-S1 URDF, M3-C2-S3→M2-C2-S7/M3-C1-S5 data, M3-C3-S6 PRIMARY REFERENCE), Latency Traps Present (M3-C1-S7, M3-C2-S2/S3, M3-C3-S2/S3/S5/S6/S7)

- [ ] T110 Insert Module Consistency Validation checklist table + Assessment Alignment Matrix in M4-C3-S7 (`docs/M4/C3/S7.md`) verifying: Sim-to-Real Adherence (full capstone achievable in simulation), Capstone Readiness (Voice→Plan→Navigate→Manipulate pipeline complete), Hardware Compliance (all tiers supported), Cross-Links Valid (M4-C1-S2→M4-C1-S3→M4-C2-S4→M1-C2-S2 VLA pipeline, M4-C3-S2→M3-C2-S5 Nav2), Latency Traps Present (M4-C1-S2, M4-C3-S5, M4-C3-S7). Include Assessment Alignment Matrix mapping all 6 assessments to section IDs.

### Cross-Module Link Injection

- [ ] T111 Insert URDF cross-link references: In M2-C1-S1 add reference to M1-C3-S1 canonical URDF path; in M3-C1-S3 add reference to M1-C3-S1 URDF for USD conversion

- [ ] T112 Insert Sim-to-Real cross-link references: In M3-C2-S3 add references to M2-C2-S7 + M3-C1-S5 for training data sources; in all 14 "Mention" sections add link to M3-C3-S6 as PRIMARY REFERENCE for weight flashing

- [ ] T113 Insert VLA→Actions cross-link references: In M4-C2-S4 add reference to M1-C2-S2 (ROS 2 Actions) with mapping table (language commands → action calls); in M4-C3-S1 add reference chain M4-C1-S2→M4-C1-S3→M4-C2-S1; in M4-C3-S2 add reference to M3-C2-S5 (Nav2); in M4-C3-S3 add reference to M3-C2-S3 (object detection)

---

## G) Code Snippets & Assets Tasks

### Code Assets Creation

- [ ] T114 [P] Create `src/snippets/M1/` directory and populate with ROS 2 example code: `joint_cmd_publisher.py`, `sensor_listener.py`, `broadcast_static_tf.py`, `ik_service.py`, `ik_client.py`, `navigate_action_server.py`, `param_node.py`, `qos_profiles.py`, `joint_state_publisher.py`, `ik_solver.py`, `imu_driver.py`

- [ ] T115 [P] Create `src/snippets/M2/` directory and populate with Gazebo/Unity example code: `pcl_filter.py`, `align_rgbd.py`, `delay_node.py`, `capture_synthetic_data.py`

- [ ] T116 [P] Create `src/snippets/M3/` directory and populate with Isaac example code: `urdf_to_usd.py`, `pytorch_to_onnx.py`, `plane_fit_ransac.py`

- [ ] T117 [P] Create `src/snippets/M4/` directory and populate with VLA example code: `audio_capture.py`, `whisper_tensorrt.py`, `command_parser.py`, `multimodal_fusion.py`, `behavior_tree.py`

### Configuration Assets

- [ ] T118 [P] Create `static/code/configs/` directory with ROS 2 config files: `nav2_params.yaml`, `robot_localization_ekf.yaml`, `ros2_control.yaml`, `slam_toolbox.yaml`

- [ ] T119 [P] Create `static/code/launch/` directory with launch files: `capstone_bringup.launch.py`, example launch file templates

### Bash Scripts

- [ ] T120 [P] Create `static/code/scripts/` directory with bash scripts: `setup-workstation.sh`, `test-gpu.sh`, `flash-jetson.sh`, `test-peripherals.sh`, `install-ros2.sh`, `install-gazebo.sh`, `install-omniverse.sh`, `install-respeaker.sh`, `tts_deployment.sh`

### Diagrams & Media

- [ ] T121 [P] Create `static/img/M1/` directory with Module 1 diagrams: humanoid sensor-actuator diagram (SVG), ROS 2 graph visualization, TF tree diagram, URDF visualization screenshot

- [ ] T122 [P] Create `static/img/M2/` directory with Module 2 media: Gazebo world screenshots, Unity HDRP rendered images, sensor visualization screenshots

- [ ] T123 [P] Create `static/img/M3/` directory with Module 3 media: Isaac Sim screenshots, USD hierarchy diagrams, VSLAM trajectory plots, PPO training curves

- [ ] T124 [P] Create `static/img/M4/` directory with Module 4 media: Capstone pipeline diagrams, behavior tree visualizations, end-to-end demo videos (embedded)

---

## H) Deployment & CI/CD Tasks (Week 1)

### GitHub Actions Workflow

- [ ] T125 Write `.github/workflows/deploy.yml` with GitHub Actions workflow: Trigger on push to `main`, steps: checkout code, setup Node.js 18.x, `npm install`, `npm run build`, deploy to `gh-pages` branch using `peaceiris/actions-gh-pages@v3`

- [ ] T126 [P] Configure repository settings for GitHub Pages: Enable GitHub Pages, set source to `gh-pages` branch, set custom domain (if applicable)

### Validation Scripts

- [ ] T127 Write `scripts/validate-structure.sh`: Bash script to verify 84 files exist at `docs/M[1-4]/C[1-3]/S[1-7].md`, check directory hierarchy matches 4×3×7, validate file naming (S1.md format). Exit with error if any files missing or misnamed.

- [ ] T128 Write `scripts/validate-latency-traps.sh`: Bash script to count latency trap placements using grep. Expected: 22 "Required Callout" (search for "⚠️ Latency Trap Warning") + 14 "Mention" (search for "See M3-C3-S6 for weight flashing"). Exit with error if counts != 36.

- [ ] T129 Write `scripts/validate-links.sh`: Bash script using `markdown-link-check` to validate internal links across all 84 Markdown files. Exit with error if broken links detected.

---

## I) QA & Consistency Tasks (Week 13)

### Final Validation

- [ ] T130 Run `scripts/validate-structure.sh` and verify output: "✅ All 84 sections found at correct paths"

- [ ] T131 Run `scripts/validate-latency-traps.sh` and verify output: "✅ 22 Required Callouts, 14 Mentions (36 total)"

- [ ] T132 Run `scripts/validate-links.sh` and verify output: "✅ All internal links valid (no broken links)"

- [ ] T133 Manually verify scope lock compliance: Review all 84 sections to confirm only brief-approved tools (ROS 2 Humble, Gazebo Harmonic, Unity, Isaac Sim/Lab/ROS, RealSense D435i/D455, Whisper, Jetson Orin, Unitree G1/Go2, Robotis OP3, AWS g5.2xlarge). No external dependencies beyond brief.

- [ ] T134 Manually verify information density: Review all 84 sections to confirm ≥1 runnable code artifact or technical procedure in each section (Constitution Article X).

- [ ] T135 Verify capstone pipeline completeness: Confirm M4-C3-S1 (Voice→Plan), M4-C3-S2 (Plan→Navigate), M4-C3-S3 (Navigate→Manipulate), M4-C3-S4 (simulation testing), M4-C3-S5 (optional physical) provide full pipeline. Test end-to-end in Gazebo or Isaac Sim.

- [ ] T136 Verify assessment alignment: Confirm M4-C3-S7 Assessment Alignment Matrix maps all 6 assessments to section IDs with no gaps (Assessment 1→M1-C1-S4/S5/S6/S7+M1-C2-S1/S3/S4, Assessment 2→M1-C3-S1/S2/S3/S4+M2-C1-S1/S3/S7, Assessment 3→M2-C2-S1-S7, Assessment 4→M3-C1-S1/S2/S3/S5+M3-C3-S2/S3/S4, Assessment 5→M1-C1-S2+M3-C2-S1/S2/S3/S5+M3-C3-S6, Assessment 6→M4-C1-S1/S2/S3+M4-C2-S1/S2/S4+M4-C3-S1/S2/S3/S4).

### Build & Deployment

- [ ] T137 Run `npm run build` and verify Docusaurus builds successfully without errors or warnings. Check output directory `build/` contains all 84 HTML pages.

- [ ] T138 Test local site: Run `npm run serve` and manually verify: (1) All 4 modules × 3 chapters × 7 sections visible in sidebar navigation, (2) Search functionality works (search for "Jetson Orin", "Latency Trap", "Capstone"), (3) Mobile responsiveness (test on mobile viewport)

- [ ] T139 Commit all changes to git with message: "feat: Complete 84-section Physical AI & Humanoid Robotics textbook. Modules: M1 (ROS 2), M2 (Gazebo/Unity), M3 (Isaac), M4 (VLA). Constitution compliant: 4×3×7 structure, scope lock, 36 latency trap placements, assessment alignment. Ready for GitHub Pages deployment."

- [ ] T140 Push to `main` branch to trigger GitHub Actions workflow (`.github/workflows/deploy.yml`). Verify workflow succeeds and deploys to `gh-pages` branch.

- [ ] T141 Verify site live at GitHub Pages URL: `https://<username>.github.io/<repo-name>/`. Test all 84 sections accessible, sidebar navigation functional, search operational.

---

## J) Documentation & Polish Tasks

### README.md

- [ ] T142 Write `README.md` in repository root with: Project title, brief description (bridges digital brain and physical body), hardware requirements table (Workstation, Edge Kit, Robot Lab tiers, Ether Lab cloud), software prerequisites (Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, Unity, Isaac Sim), setup instructions (`npm install`, `npm run build`, `npm run start`), deployment instructions (GitHub Pages), contribution guidelines, license (MIT or as specified)

### Additional Assets

- [ ] T143 [P] Create `static/datasets/` with sample synthetic datasets: `gazebo_synthetic_sample.zip` (from M2-C2-S7), `isaac_replicator_sample.zip` (from M3-C1-S5)

- [ ] T144 [P] Create MDX component `src/components/LatencyTrapCallout.jsx` for reusable latency trap warning block (optional enhancement for consistent styling)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (T001-T006)**: Docusaurus initialization - No dependencies
- **Phase 2 (T007-T020)**: Content scaffolding - Depends on Phase 1 (Docusaurus project must exist)
- **Module 1 (T021-T041)**: Week 2-3 writing - Depends on Phase 2 (directory structure + placeholders must exist)
  - **T041 (M1-C3-S7)**: Module 1 Consistency Check - Blocks Module 2 start
- **Module 2 (T042-T062)**: Week 4-6 writing - Depends on T041 (M1 complete)
  - **T042 (M2-C1-S1)**: Depends on T035 (M1-C3-S1 URDF canonical path)
  - **T062 (M2-C3-S7)**: Module 2 Consistency Check - Blocks Module 3 start
- **Module 3 (T063-T083)**: Week 7-9 writing - Depends on T062 (M2 complete)
  - **T065 (M3-C1-S3)**: Depends on T035 (M1-C3-S1 URDF for USD conversion)
  - **T072 (M3-C2-S3)**: Depends on T055 (M2-C2-S7) and T067 (M3-C1-S5) for synthetic data
  - **T083 (M3-C3-S7)**: Module 3 Consistency Check - Blocks Module 4 start
- **Module 4 (T084-T104)**: Week 10-12 writing - Depends on T083 (M3 complete)
  - **T086 (M4-C1-S3)**: Depends on T085 (M4-C1-S2 Whisper output)
  - **T092 (M4-C2-S2)**: Depends on T074 (M3-C2-S5 Nav2)
  - **T094 (M4-C2-S4)**: Depends on T029 (M1-C2-S2 Actions)
  - **T099 (M4-C3-S2)**: Depends on T074 (M3-C2-S5 Nav2)
  - **T100 (M4-C3-S3)**: Depends on T072 (M3-C2-S3 object detection)
  - **T104 (M4-C3-S7)**: Final Consistency Check - Blocks deployment
- **Injection (T105-T113)**: Constitution enforcement - Can run in parallel with writing tasks or after completion
- **Code Assets (T114-T124)**: Support files - Can run in parallel with writing tasks
- **Deployment (T125-T129)**: CI/CD setup - Depends on Phase 1, can run early (Week 1)
- **QA (T130-T141)**: Final validation - Depends on all 84 sections complete (T104)
- **Polish (T142-T144)**: Documentation - Can run in parallel with QA

### Parallel Opportunities

**Week 1 (Scaffolding)**:
- T001-T006 (Docusaurus init), T007-T014 (directory creation), T125-T129 (CI/CD setup) can run in parallel
- T015-T020 (sidebars + frontmatter) depends on T007-T014

**Week 2 (M1-C1)**:
- All 7 writing tasks (T021-T027) can run in parallel (independent sections, different files)

**Week 3 (M1-C2/C3)**:
- T028-T034 (M1-C2 services/actions/parameters/launch/messages/QoS/rosbag) can run in parallel
- T035-T040 (M1-C3 URDF/Xacro/FK/IK/IMU/collisions) can run in parallel
- T041 (M1-C3-S7 Consistency Check) depends on T028-T040 completion

**Week 4-12 (M2, M3, M4)**:
- Within each chapter, most section writing tasks can run in parallel (different files)
- Cross-link tasks (T111-T113) can run after relevant sections written
- Module Consistency Checks (T041, T062, T083, T104) must complete before next module starts

**Code Assets (T114-T124)**:
- All 11 tasks can run in parallel (independent files in different directories)

**Injection Tasks (T105-T113)**:
- T105 (Required Callout insertion) + T106 (Mention insertion) can run in parallel
- T107-T110 (Module Consistency checklists) can run in parallel after respective modules written

---

## Implementation Strategy

### MVP First (Milestone 0 + 1)

1. Complete Phase 1: Docusaurus init (T001-T006)
2. Complete Phase 2: Scaffolding (T007-T020)
3. **Complete Module 1** (T021-T041): 21 sections
4. **STOP and VALIDATE**: Run T130-T132 on Module 1 only; verify M1-C3-S7 checklist passes
5. Build site: `npm run build` (should succeed with M1 complete, M2/M3/M4 placeholders)

### Incremental Delivery

1. **Milestone 0** (Week 1): Scaffolding → `npm run build` succeeds with 84 placeholders
2. **Milestone 1** (Weeks 2-3): Module 1 → Test M1-C3-S7 checklist → Freeze M1
3. **Milestone 2** (Weeks 4-6): Module 2 → Test M2-C3-S7 checklist → Freeze M2
4. **Milestone 3** (Weeks 7-9): Module 3 → Test M3-C3-S7 checklist → Freeze M3
5. **Milestone 4** (Weeks 10-12): Module 4 → Test M4-C3-S7 checklist → Freeze M4
6. **Milestone 5** (Week 13): Final validation (T130-T141) → GitHub Pages deploy

### Parallel Team Strategy

With multiple writers:

1. **Week 1**: Team completes scaffolding together (T001-T020)
2. **Weeks 2-3**:
   - Writer A: M1-C1 (T021-T027)
   - Writer B: M1-C2 (T028-T034)
   - Writer C: M1-C3 (T035-T041)
3. **Weeks 4-6**:
   - Writer A: M2-C1 (T042-T048)
   - Writer B: M2-C2 (T049-T055)
   - Writer C: M2-C3 (T056-T062)
4. **Continue pattern for M3 and M4**
5. **Week 13**: Team validates together (T130-T141)

---

## Task Summary

**Total Tasks**: 144

**Breakdown by Category**:
- Repository & Docusaurus Setup (A): 6 tasks (T001-T006)
- Content Scaffolding (B): 14 tasks (T007-T020)
- Module 1 Writing (C): 21 tasks (T021-T041)
- Module 2 Writing (D): 21 tasks (T042-T062)
- Module 3 Writing (E): 21 tasks (T063-T083)
- Module 4 Writing (F): 21 tasks (T084-T104)
- Injection Tasks (G): 9 tasks (T105-T113)
- Code Snippets & Assets (H): 11 tasks (T114-T124)
- Deployment & CI/CD (I): 5 tasks (T125-T129)
- QA & Consistency (J): 12 tasks (T130-T141)
- Documentation & Polish (K): 3 tasks (T142-T144)

**Parallel Opportunities**: 89 tasks marked [P] can run in parallel (61% parallelizable)

**Critical Path**: Scaffolding (T001-T020) → M1 Writing (T021-T041) → M1 Check (T041) → M2 Writing (T042-T062) → M2 Check (T062) → M3 Writing (T063-T083) → M3 Check (T083) → M4 Writing (T084-T104) → M4 Check (T104) → QA (T130-T141) → Deploy (T140-T141)

**Suggested MVP Scope**: Milestone 0 + Milestone 1 (Scaffolding + Module 1 complete, 21/84 sections)

---

**END OF TASK LIST**
