---
id: self-assessment
title: Self-Assessment Quizzes
sidebar_position: 106
keywords: ['quiz', 'assessment', 'test', 'review', 'knowledge-check']
---

# 📝 Self-Assessment Quizzes

Test your understanding of key concepts from each module. Answers are provided at the bottom of each section.

---

## Module 1: Foundations

### Quiz 1.1: Linux & Development Environment

**Q1.** What is the primary advantage of using Ubuntu 22.04 for ROS 2 development?

- A) It's free and open source
- B) It's a Tier 1 supported platform for ROS 2 Humble
- C) It has the best graphics performance
- D) It runs faster than other operating systems

**Q2.** Which command lists all currently running ROS 2 nodes?

- A) `ros2 topic list`
- B) `ros2 node list`
- C) `ros2 service list`
- D) `ros2 run list`

**Q3.** In ROS 2, what is the purpose of a QoS (Quality of Service) profile?

- A) To measure code quality
- B) To configure message delivery guarantees and behavior
- C) To set CPU priority for nodes
- D) To encrypt messages

**Q4.** What happens when you set QoS reliability to "best effort"?

- A) Messages are guaranteed to be delivered
- B) Messages may be dropped but with lower latency
- C) Messages are encrypted
- D) Messages are compressed

**Q5.** Which file format is used to describe robot kinematics in ROS 2?

- A) JSON
- B) YAML
- C) URDF/XACRO
- D) XML Schema

<details>
<summary>📋 Answers - Quiz 1.1</summary>

1. **B** - Ubuntu 22.04 is a Tier 1 supported platform, meaning ROS 2 Humble is extensively tested and officially supported on it.
2. **B** - `ros2 node list` shows all active nodes in the ROS 2 system.
3. **B** - QoS profiles configure how messages are delivered, including reliability, durability, and deadline settings.
4. **B** - Best effort QoS prioritizes low latency over guaranteed delivery; messages may be dropped under network congestion.
5. **C** - URDF (Unified Robot Description Format) and XACRO (XML Macros) describe robot structure, kinematics, and dynamics.

**Score:** ___/5
</details>

### Quiz 1.2: ROS 2 Concepts

**Q1.** What is the difference between a ROS 2 topic and a service?

- A) Topics are synchronous, services are asynchronous
- B) Topics use publish-subscribe (async), services use request-response (sync)
- C) Topics are faster than services
- D) Services can only send integers

**Q2.** In TF2, what does the transform from "base_link" to "camera_link" represent?

- A) The camera's internal parameters
- B) The position and orientation of the camera relative to the robot base
- C) The camera's image resolution
- D) The network connection to the camera

**Q3.** What is the purpose of a ROS 2 launch file?

- A) To compile the code
- B) To start multiple nodes with configured parameters
- C) To download packages
- D) To create Docker containers

**Q4.** Which DDS implementation is recommended for low-latency robotics applications?

- A) Fast DDS (default)
- B) Cyclone DDS
- C) OpenDDS
- D) CoreDX

**Q5.** What does the `colcon build --symlink-install` flag do?

- A) Compresses the build output
- B) Creates symbolic links so source changes take effect without rebuilding
- C) Installs to the system directory
- D) Builds with debug symbols

<details>
<summary>📋 Answers - Quiz 1.2</summary>

1. **B** - Topics use publish-subscribe for asynchronous one-to-many communication; services use request-response for synchronous one-to-one communication.
2. **B** - TF2 transforms represent the spatial relationship (translation + rotation) between coordinate frames.
3. **B** - Launch files orchestrate starting multiple nodes with specific configurations, parameters, and remappings.
4. **B** - Cyclone DDS is often recommended for robotics due to its lower latency and deterministic behavior.
5. **B** - Symlink install creates links to source files, allowing Python changes to take effect immediately without rebuilding.

**Score:** ___/5
</details>

---

## Module 2: Simulation

### Quiz 2.1: Physics Simulation

**Q1.** Why is domain randomization important for sim-to-real transfer?

- A) It makes simulations run faster
- B) It exposes the model to variations, improving real-world robustness
- C) It reduces GPU memory usage
- D) It simplifies the physics calculations

**Q2.** What is the "reality gap" in robotics simulation?

- A) The difference between simulation speed and real-time
- B) The discrepancy between simulated and real-world behavior
- C) The gap between robot and human intelligence
- D) The time delay in sensor data

**Q3.** In Gazebo, what is the purpose of the SDF file?

- A) To define robot joint controllers
- B) To describe the simulation world, including models and physics
- C) To configure network settings
- D) To store sensor calibration data

**Q4.** What physics parameter most affects contact stability in simulation?

- A) Gravity magnitude
- B) Solver iterations and timestep
- C) Light intensity
- D) Camera frame rate

**Q5.** Which simulator provides GPU-accelerated parallel environments for RL training?

- A) Gazebo Classic
- B) PyBullet
- C) Isaac Gym / Isaac Sim
- D) V-REP

<details>
<summary>📋 Answers - Quiz 2.1</summary>

1. **B** - Domain randomization varies simulation parameters (textures, lighting, physics) during training, making policies robust to real-world variations.
2. **B** - The reality gap refers to differences between simulation and reality that cause policies trained in simulation to fail on real robots.
3. **B** - SDF (Simulation Description Format) defines the complete simulation environment including models, physics properties, and world configuration.
4. **B** - Solver iterations and timestep directly affect contact resolution accuracy and stability in physics simulation.
5. **C** - Isaac Gym and Isaac Sim from NVIDIA provide GPU-accelerated physics for running thousands of parallel environments.

**Score:** ___/5
</details>

### Quiz 2.2: Sensor Simulation

**Q1.** What is the purpose of adding noise to simulated sensors?

- A) To slow down the simulation
- B) To match real sensor behavior and improve transfer
- C) To reduce GPU memory usage
- D) To encrypt sensor data

**Q2.** Which type of depth sensor noise is most challenging for sim-to-real transfer?

- A) Gaussian noise
- B) Edge artifacts and reflections on specular surfaces
- C) Fixed offset bias
- D) Temperature drift

**Q3.** What is camera intrinsic calibration?

- A) Determining camera position on the robot
- B) Determining internal parameters like focal length and distortion
- C) Setting exposure and white balance
- D) Calibrating network latency

**Q4.** Why is IMU bias instability important for long-duration navigation?

- A) It affects battery life
- B) Bias drift accumulates over time, causing position errors
- C) It determines sensor sampling rate
- D) It affects wireless communication

**Q5.** What is the Allan Variance plot used for?

- A) Comparing different robot models
- B) Characterizing noise types in inertial sensors
- C) Measuring network latency
- D) Evaluating path planning efficiency

<details>
<summary>📋 Answers - Quiz 2.2</summary>

1. **B** - Adding realistic noise to simulated sensors helps models learn to handle noise they'll encounter on real hardware.
2. **B** - Edge artifacts, missing returns on reflective surfaces, and multi-path effects are challenging to simulate accurately.
3. **B** - Intrinsic calibration determines the camera's internal parameters: focal length (fx, fy), principal point (cx, cy), and distortion coefficients.
4. **B** - Bias instability causes IMU measurements to drift over time; this drift accumulates when integrating to get position.
5. **B** - Allan Variance analysis identifies different noise components in IMU data: white noise, bias instability, and random walk.

**Score:** ___/5
</details>

---

## Module 3: Locomotion & Perception

### Quiz 3.1: Motion Planning

**Q1.** What does RRT stand for, and what type of planning does it perform?

- A) Real-Time Trajectory - time-optimal planning
- B) Rapidly-exploring Random Tree - sampling-based motion planning
- C) Recursive Robot Transform - kinematic calculation
- D) Reliable Robotic Teleoperation - remote control

**Q2.** What is the Jacobian matrix used for in robotics?

- A) Image processing
- B) Relating joint velocities to end-effector velocities
- C) Network communication
- D) Battery monitoring

**Q3.** In inverse kinematics, what does "multiple solutions" mean?

- A) The robot has multiple joints
- B) Several different joint configurations can achieve the same end-effector pose
- C) The algorithm runs multiple times
- D) Multiple robots work together

**Q4.** What is the advantage of TracIK over KDL for inverse kinematics?

- A) TracIK is faster
- B) TracIK has higher success rate by combining multiple methods
- C) TracIK uses less memory
- D) TracIK supports more robot types

**Q5.** What is a costmap in navigation?

- A) A financial planning tool
- B) A 2D grid representing traversal difficulty and obstacles
- C) A network routing table
- D) A camera calibration matrix

<details>
<summary>📋 Answers - Quiz 3.1</summary>

1. **B** - RRT (Rapidly-exploring Random Tree) is a sampling-based algorithm that builds a tree exploring configuration space.
2. **B** - The Jacobian relates joint-space velocities to task-space (end-effector) velocities: ẋ = J(q)q̇
3. **B** - A robot arm may have multiple valid joint configurations (e.g., elbow up vs. elbow down) that place the end-effector at the same pose.
4. **B** - TracIK combines KDL with SQP optimization, achieving higher success rates especially near singularities.
5. **B** - A costmap is a grid where each cell has a cost value; obstacles have infinite cost, clear space has low cost, with inflation around obstacles.

**Score:** ___/5
</details>

### Quiz 3.2: Perception

**Q1.** What is the output of an object detection model like YOLO?

- A) A single class label
- B) Bounding boxes with class labels and confidence scores
- C) A segmentation mask
- D) A depth map

**Q2.** What does Non-Maximum Suppression (NMS) do?

- A) Increases detection confidence
- B) Removes redundant overlapping detections
- C) Converts RGB to grayscale
- D) Normalizes input images

**Q3.** What is the purpose of TensorRT in robot perception?

- A) To train models from scratch
- B) To optimize and accelerate inference on NVIDIA GPUs
- C) To collect training data
- D) To annotate images

**Q4.** What is point cloud segmentation used for in manipulation?

- A) Making pretty visualizations
- B) Identifying graspable objects and surfaces
- C) Compressing data for transmission
- D) Encrypting sensor data

**Q5.** What does quantization (FP16/INT8) trade off?

- A) Speed vs. model size
- B) Precision vs. inference speed and memory
- C) Training time vs. accuracy
- D) Color depth vs. resolution

<details>
<summary>📋 Answers - Quiz 3.2</summary>

1. **B** - Object detectors output bounding boxes (x, y, width, height), class labels, and confidence scores for each detection.
2. **B** - NMS removes duplicate detections of the same object by keeping only the highest-confidence box when boxes overlap significantly.
3. **B** - TensorRT optimizes neural networks for NVIDIA GPUs through kernel fusion, precision calibration, and other optimizations.
4. **B** - Segmenting point clouds allows robots to identify individual objects, surfaces, and grasp points for manipulation.
5. **B** - Quantization reduces numerical precision (32-bit to 16-bit or 8-bit), trading some accuracy for faster inference and lower memory usage.

**Score:** ___/5
</details>

### Quiz 3.3: Bipedal Locomotion

**Q1.** What is the Zero Moment Point (ZMP)?

- A) The robot's center of mass
- B) The point where ground reaction forces produce zero horizontal moment
- C) The lowest point of the robot
- D) The balance point of the foot

**Q2.** Which balance strategy is used for small perturbations on firm ground?

- A) Stepping strategy
- B) Hip strategy
- C) Ankle strategy
- D) Arm waving strategy

**Q3.** What is the Linear Inverted Pendulum Model (LIPM) used for?

- A) Image classification
- B) Simplified bipedal walking dynamics for gait planning
- C) Network load balancing
- D) Battery charge estimation

**Q4.** In reinforcement learning for locomotion, what is curriculum learning?

- A) Learning from textbooks
- B) Progressively increasing task difficulty during training
- C) Learning in a virtual classroom
- D) Following a predefined curriculum

**Q5.** What is the Cost of Transport (CoT) metric?

- A) The price of robot shipping
- B) Energy efficiency: energy used per unit mass per unit distance
- C) Network bandwidth cost
- D) Time to complete a task

<details>
<summary>📋 Answers - Quiz 3.3</summary>

1. **B** - ZMP is where the sum of gravity and inertia forces produces zero moment around the horizontal axes; keeping ZMP in the support polygon ensures stability.
2. **C** - Ankle strategy uses ankle torques to make small corrections, effective for perturbations up to about 5cm center of mass displacement.
3. **B** - LIPM simplifies bipedal dynamics by treating the robot as an inverted pendulum, enabling analytical gait generation.
4. **B** - Curriculum learning starts with easy tasks (flat ground, slow speed) and progressively increases difficulty (slopes, speed, perturbations).
5. **B** - CoT = Energy / (Mass × Distance × Gravity), measuring how efficiently the robot moves; lower is better.

**Score:** ___/5
</details>

---

## Module 4: System Integration

### Quiz 4.1: Human-Robot Interaction

**Q1.** What is the purpose of intent recognition in voice interfaces?

- A) To verify user identity
- B) To understand the goal behind a user's command
- C) To translate languages
- D) To improve audio quality

**Q2.** Why should speech recognition run locally on the robot?

- A) To save money on cloud services
- B) To minimize latency for responsive interaction
- C) To use more GPU memory
- D) To improve accuracy

**Q3.** What is entity extraction in natural language understanding?

- A) Removing unnecessary words
- B) Identifying specific items like objects, locations, or names from text
- C) Converting text to speech
- D) Encrypting sensitive information

**Q4.** What does "grounding" mean in the context of language for robotics?

- A) Electrically grounding the robot
- B) Connecting abstract language to physical world entities the robot can perceive
- C) Stabilizing the robot's balance
- D) Training the language model

**Q5.** Which model is commonly used for real-time speech-to-text on edge devices?

- A) GPT-4
- B) Whisper
- C) DALL-E
- D) Stable Diffusion

<details>
<summary>📋 Answers - Quiz 4.1</summary>

1. **B** - Intent recognition determines what the user wants to accomplish (navigate, manipulate, query, etc.) from their spoken command.
2. **B** - Local processing avoids 100-300ms network round-trip latency, enabling natural conversation flow.
3. **B** - Entity extraction identifies specific items in text: "Put the **red cup** on the **kitchen table**" → objects and locations.
4. **B** - Grounding connects language ("the cup near the computer") to specific physical objects the robot can perceive and interact with.
5. **B** - OpenAI's Whisper model provides accurate speech recognition and can run locally on edge devices.

**Score:** ___/5
</details>

### Quiz 4.2: Task Planning

**Q1.** What is the advantage of using behavior trees over finite state machines?

- A) Behavior trees are faster to execute
- B) Behavior trees are more modular and easier to modify
- C) Behavior trees use less memory
- D) Behavior trees don't require programming

**Q2.** In a behavior tree, what does a Sequence node do?

- A) Runs children in parallel
- B) Runs children in order, succeeding only if all succeed
- C) Runs the first successful child
- D) Randomly selects a child

**Q3.** What is the role of LLMs in robot task planning?

- A) To directly control robot motors
- B) To generate high-level plans from natural language instructions
- C) To process sensor data
- D) To manage network connections

**Q4.** What is error recovery in manipulation systems?

- A) Fixing code bugs
- B) Detecting failures and taking corrective actions to continue tasks
- C) Recovering deleted files
- D) Restarting the computer

**Q5.** What is the purpose of a "Blackboard" in behavior trees?

- A) For taking notes during meetings
- B) A shared data structure for nodes to communicate state
- C) A debugging visualization tool
- D) A training aid for students

<details>
<summary>📋 Answers - Quiz 4.2</summary>

1. **B** - Behavior trees are modular, reusable, and easier to modify than state machines which can become tangled "spaghetti" for complex behaviors.
2. **B** - A Sequence node runs children left-to-right, returning success only if all children succeed; it returns failure on the first child failure.
3. **B** - LLMs can translate natural language commands into structured task plans, leveraging their reasoning capabilities.
4. **B** - Error recovery involves detecting when actions fail (grasp failed, navigation blocked) and executing recovery behaviors.
5. **B** - The Blackboard is a shared memory space where behavior tree nodes can read/write data to coordinate their actions.

**Score:** ___/5
</details>

---

## Comprehensive Final Assessment

### Mixed Topics Quiz

**Q1.** Which of these tasks should NOT be offloaded to the cloud?

- A) LLM-based task planning
- B) Model training
- C) Real-time balance control
- D) Map storage

**Q2.** What is the primary purpose of a digital twin in robotics?

- A) Creating robot duplicates
- B) A virtual replica for monitoring, simulation, and testing
- C) Backup power supply
- D) Network redundancy

**Q3.** In sim-to-real transfer, what helps bridge the reality gap?

- A) Using more powerful GPUs
- B) Domain randomization and accurate sensor noise models
- C) Faster network connections
- D) More training data only

**Q4.** What is the typical latency budget for voice-to-action in humanoid robots?

- A) 5 seconds
- B) 500ms - 2 seconds
- C) 10 milliseconds
- D) 1 minute

**Q5.** Which metric indicates a locomotion policy is energy efficient?

- A) High velocity
- B) Low Cost of Transport (CoT)
- C) Many training episodes
- D) Large neural network size

**Q6.** What does "action chunking" mean in diffusion policies?

- A) Breaking actions into smaller pieces
- B) Predicting sequences of future actions, not just single steps
- C) Compressing action data
- D) Grouping similar actions together

**Q7.** In fleet management, what triggers task reassignment?

- A) User request
- B) Robot failure or going offline
- C) Time of day
- D) Weather conditions

**Q8.** What is the purpose of the inflation layer in a costmap?

- A) To make the map larger
- B) To add a safety buffer around obstacles
- C) To compress the data
- D) To improve visual appearance

**Q9.** Why use DDIM instead of DDPM for diffusion policy inference?

- A) DDIM is more accurate
- B) DDIM requires fewer denoising steps, enabling faster inference
- C) DDIM uses less GPU memory
- D) DDIM trains faster

**Q10.** What certification is most aligned with this textbook's ROS 2 content?

- A) AWS Solutions Architect
- B) ROS 2 Developer Certification
- C) Google Cloud ML Engineer
- D) Cisco Networking

<details>
<summary>📋 Answers - Final Assessment</summary>

1. **C** - Balance control requires 1-5ms response times; cloud latency (50-200ms) is far too slow for safety-critical control.
2. **B** - A digital twin is a virtual replica of the physical system, enabling remote monitoring, testing, and prediction.
3. **B** - Domain randomization and accurate sensor/physics modeling help policies trained in simulation work on real robots.
4. **B** - Voice interaction should feel responsive; 500ms-2s is the typical acceptable latency for voice-to-action.
5. **B** - Lower Cost of Transport means less energy used per unit mass per unit distance traveled.
6. **B** - Action chunking predicts multiple future actions at once, amortizing inference cost over several control steps.
7. **B** - Fleet managers reassign tasks when robots fail or go offline to ensure task completion.
8. **B** - The inflation layer adds cost around obstacles proportional to distance, keeping the robot a safe distance away.
9. **B** - DDIM enables deterministic sampling with fewer steps (10-20 vs. 100+), critical for real-time control.
10. **B** - The ROS 2 Developer Certification directly aligns with the ROS 2 skills taught throughout this textbook.

**Score:** ___/10
</details>

---

## Scoring Guide

| Score Range | Assessment | Recommendation |
|-------------|------------|----------------|
| 90-100% | **Expert** | Ready for advanced topics and capstone |
| 75-89% | **Proficient** | Review specific weak areas |
| 60-74% | **Developing** | Revisit relevant sections before proceeding |
| Below 60% | **Needs Review** | Study the material more thoroughly |

---

## Knowledge Gaps Analysis

After completing the quizzes, identify your weak areas:

### Module 1 Gaps
- [ ] Need to review ROS 2 communication patterns
- [ ] Need to practice with QoS settings
- [ ] Need to understand TF2 better

### Module 2 Gaps
- [ ] Need to review physics simulation concepts
- [ ] Need to understand domain randomization
- [ ] Need to practice sensor calibration

### Module 3 Gaps
- [ ] Need to review motion planning algorithms
- [ ] Need to understand perception pipeline
- [ ] Need to study locomotion control

### Module 4 Gaps
- [ ] Need to review NLU concepts
- [ ] Need to understand behavior trees
- [ ] Need to practice system integration

---

:::tip Study Strategy
1. Take quizzes **before** reading the module to identify what you don't know
2. Focus study time on weak areas
3. Retake quizzes **after** completing the module
4. Aim for 90%+ before moving to the next module
:::

:::info Connection to Capstone
Achieving 80%+ across all quizzes indicates readiness to begin the capstone project. Lower scores suggest reviewing specific modules first.
:::

