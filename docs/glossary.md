---
id: glossary
title: Glossary of Terms
sidebar_position: 100
keywords: ['glossary', 'terms', 'definitions', 'reference', 'dictionary']
---

# 📚 Glossary of Terms

This comprehensive glossary defines 200+ technical terms used throughout the Physical AI and Humanoid Robotics textbook. Terms are organized alphabetically for quick reference.

:::tip Quick Navigation
Use `Ctrl+F` (or `Cmd+F` on Mac) to search for specific terms.
:::

---

## A

### Accelerometer
An inertial sensor that measures linear acceleration along one or more axes. In humanoid robots, accelerometers detect body motion, impacts, and orientation changes relative to gravity.

### Action Primitive
A fundamental, reusable robot behavior (e.g., "reach", "grasp", "release") that can be composed into complex tasks. Action primitives form the building blocks of manipulation pipelines.

### Action Space
The set of all possible actions an agent can take in a reinforcement learning environment. For humanoid robots, this typically includes joint torques, velocities, or position targets.

### Actor-Critic Architecture
A reinforcement learning architecture combining a policy network (actor) that selects actions with a value network (critic) that evaluates state quality. Used in PPO and SAC algorithms for locomotion control.

### Affordance
The potential actions an object offers to an agent. A cup affords grasping; a door affords opening. Affordance detection enables robots to understand how to interact with objects.

### AHRS (Attitude and Heading Reference System)
A sensor fusion system combining accelerometer, gyroscope, and magnetometer data to estimate 3D orientation. Essential for humanoid balance and navigation.

### Aleatoric Uncertainty
Irreducible uncertainty inherent in data (e.g., sensor noise). Contrasts with epistemic uncertainty, which can be reduced with more data.

### Allan Variance
A statistical method for characterizing noise in inertial sensors (IMUs). Used to identify bias instability, random walk, and other error sources during calibration.

### Analytical IK (Inverse Kinematics)
Closed-form mathematical solutions for computing joint angles from end-effector poses. Faster than numerical IK but only available for specific robot geometries.

### Ankle Strategy
A balance recovery strategy where the humanoid adjusts ankle torques to maintain center of mass over the support polygon. Effective for small perturbations on firm surfaces.

### Anisotropic Friction
Friction that varies with direction, important for simulating materials like brushes or textured surfaces in physics engines.

---

## B

### Balance Control
The system responsible for maintaining humanoid stability by adjusting joint torques and body posture in response to disturbances and during locomotion.

### Beamforming
A signal processing technique that combines multiple microphone signals to focus on sound from a specific direction while suppressing noise. Used in robot speech recognition.

### Behavior Tree
A hierarchical control architecture using nodes (Sequence, Selector, Parallel) to compose complex robot behaviors. More modular and maintainable than finite state machines.

### Benchmark Dataset
A standardized dataset with ground truth labels used to evaluate and compare algorithm performance. Examples: COCO for detection, KITTI for SLAM.

### Bias Instability
The minimum point on an Allan Variance plot, representing the best achievable stability of an inertial sensor. Critical for long-term navigation accuracy.

### Binary Bayes Filter
A probabilistic filter that estimates binary states (e.g., occupied/free) using sensor observations. Foundation for occupancy grid mapping.

### Blackboard
A shared data structure in Behavior Trees that allows nodes to communicate state information without direct coupling.

### Blocking Call
A function call that halts execution until completion. In ROS 2, synchronous service calls are blocking; prefer async calls in real-time systems.

### Bounding Box
A rectangular region (x1, y1, x2, y2) that localizes a detected object within an image. Output format for object detection models like [YOLO](https://docs.ultralytics.com/models/yolo11/).

---

## C

### Calibration
The process of determining sensor parameters (intrinsics, extrinsics, noise characteristics) to ensure accurate measurements. Essential for perception and control.

### Camera Intrinsics
Internal camera parameters: focal length (fx, fy), principal point (cx, cy), and distortion coefficients. Required for 3D reconstruction.

### Callback
A function registered to be called when an event occurs (e.g., message received, timer fires). Core pattern in ROS 2 node programming.

### Cardinality
The number of elements in a set. In robotics, used to describe the size of action/observation spaces or the number of detected objects.

### CLIP Model
Contrastive Language-Image Pre-training model from OpenAI. Enables zero-shot image classification and vision-language understanding for object grounding.

### Clipped Surrogate Objective
The loss function in PPO that limits policy updates to prevent destructive large changes. Key to stable reinforcement learning training.

### Cognitive Planning
High-level task planning that reasons about goals, preconditions, and effects. Often implemented with LLMs or classical planners (PDDL).

### Collision Geometry
Simplified shapes (boxes, spheres, meshes) used for collision detection in physics simulation. Simpler than visual geometry for performance.

### Colcon
The build tool for ROS 2 workspaces. Compiles packages and manages dependencies with commands like `colcon build`.

### Complementary Filter
A simple sensor fusion algorithm that combines high-frequency gyroscope data with low-frequency accelerometer data for orientation estimation.

### Confidence Score
A probability value (0-1) indicating model certainty about a prediction. Used to filter low-confidence detections.

### Contact Model
The mathematical representation of how objects interact when touching, including normal forces, friction, and deformation.

### Context Memory
Short-term storage of conversation history and environment state for multi-turn dialogue and task continuity.

### Control Frequency
The rate (Hz) at which the control loop executes. Humanoid balance typically requires 200-1000 Hz; manipulation can work at 100-500 Hz.

### Convex Hull
The smallest convex shape containing a set of points. Used for simplified collision geometry and grasp analysis.

### Coordinate Frame
A reference system defining position and orientation. ROS uses right-handed frames with conventions like base_link, odom, and map.

### Costmap
A 2D grid representation of traversability costs for navigation. Combines static map, obstacle layer, and inflation layer.

### Coulomb Friction
The force opposing relative motion between surfaces, proportional to normal force: F = μN. Foundation of contact simulation.

### CUDA
NVIDIA's parallel computing platform enabling GPU acceleration for deep learning and physics simulation.

### Curriculum Learning
A training strategy that progressively increases task difficulty, helping policies learn complex behaviors incrementally.

---

## D

### Damping
The dissipation of energy in a system, preventing oscillation. Joint damping affects how quickly movements settle.

### DDS (Data Distribution Service)
The middleware underlying ROS 2 communication, providing publish-subscribe messaging with configurable QoS.

### Deadline
A timing constraint specifying when data must be delivered. Critical for real-time robotics; missed deadlines can cause instability.

### Deep Learning
Machine learning using neural networks with multiple layers. Powers modern perception (CNNs), language (transformers), and control (RL).

### Deictic Reference
Language that refers to the context of the utterance, like "this", "here", "now". Requires grounding in the physical world.

### Demonstration
Expert-provided examples of desired behavior, used in imitation learning to train policies without explicit reward design.

### Depth Camera
A sensor that measures distance per pixel, producing depth images. Technologies include stereo, structured light, and ToF.

### Depth Noise
Sensor-specific error patterns in depth measurements, including shot noise, quantization noise, and systematic biases.

### Dialogue State Tracking
Maintaining the current state of a conversation, including user intents, extracted entities, and conversation history.

### Digital Twin
A virtual replica of a physical system that mirrors its state and behavior, enabling simulation, monitoring, and prediction.

### Diffusion Policy
An emerging approach using denoising diffusion models for robot policy learning, showing strong performance in manipulation tasks.

### Domain Adaptation
Techniques for transferring models trained in one domain (e.g., simulation) to perform well in another (e.g., real world).

### Domain Randomization
Varying simulation parameters (lighting, textures, physics) during training to improve real-world transfer robustness.

### Durability
A DDS QoS setting determining how long messages persist. "Transient local" keeps messages for late-joining subscribers.

---

## E

### Echo Cancellation
Removing the robot's own voice from microphone input to prevent feedback during speech interaction.

### Edge Computing
Processing data locally on embedded devices (e.g., Jetson) rather than in the cloud, reducing latency for real-time robotics.

### EKF (Extended Kalman Filter)
A nonlinear state estimation algorithm linearizing around current estimates. Widely used for robot localization and sensor fusion.

### Embodied Intelligence
AI that learns through physical interaction with the environment, integrating perception, action, and learning.

### Emergency Stop (E-Stop)
A safety mechanism that immediately halts robot motion. Required for all physical robot deployments.

### End-to-End Learning
Training a single model from raw inputs (images, audio) directly to outputs (actions) without hand-engineered intermediate representations.

### Entity
A named piece of information extracted from text, such as locations ("kitchen"), objects ("cup"), or people ("John").

### Entity Linking
Connecting extracted text entities to entries in a knowledge base, resolving ambiguity (e.g., "apple" → fruit or company).

### Entropy Bonus
A regularization term in RL that encourages exploration by rewarding policy randomness during training.

### Epistemic Uncertainty
Uncertainty from limited knowledge, reducible with more data. Contrasts with aleatoric uncertainty.

### Episode
A single run of a task from start to termination in reinforcement learning. Policies are trained over many episodes.

### Extrinsic Calibration
Determining the transformation (position and orientation) between different sensors or between sensors and robot links.

---

## F

### Failure Mode
A specific way a system can fail. Identifying failure modes enables designing appropriate recovery behaviors.

### Fault Tolerance
The ability of a system to continue operating despite component failures, through redundancy or graceful degradation.

### FCL (Flexible Collision Library)
An open-source library for collision detection and proximity queries, used by MoveIt2 for motion planning.

### Feature Alignment
Domain adaptation technique that learns representations where source and target domain features have similar distributions.

### Feature Descriptor
A compact numerical representation of a visual feature (e.g., SIFT, ORB) enabling matching across images.

### Feedback Loop
A control system where output is measured and compared to the desired setpoint to compute corrective action.

### Fine-tuning
Adapting a pre-trained model to a specific task by training on task-specific data with a lower learning rate.

### Force Closure
A grasp property where contact forces can resist arbitrary external wrenches, ensuring stable manipulation.

### Force Control
Controlling robot motion based on measured or estimated forces, essential for compliant manipulation.

### FP16 Precision
Half-precision floating-point format using 16 bits. Reduces memory and increases inference speed with minimal accuracy loss.

### Friction Cone
The set of force directions that can be applied at a contact point without slipping, determined by friction coefficient.

---

## G

### Gait Cycle
One complete sequence of leg movements during walking, from heel strike to the next heel strike of the same foot.

### Gazebo
An open-source robotics simulator providing physics, sensors, and 3D visualization. Gazebo Harmonic is the latest version.

### Gaze Estimation
Determining where a person is looking, used for human-robot interaction and attention-aware behavior.

### Gesture Classification
Recognizing hand or body gestures from sensor data for non-verbal human-robot communication.

### Global Planner
A path planning algorithm that finds routes through the entire known environment (e.g., A*, Dijkstra, RRT).

### Goal Handle
A ROS 2 action server construct representing an accepted goal, allowing status queries and result retrieval.

### GPU Acceleration
Using graphics processing units for parallel computation, dramatically speeding up deep learning and physics simulation.

### Graceful Degradation
System design where partial failures reduce capability without complete system failure.

### Gradient Reversal
A domain adaptation technique that trains features to be discriminative for the main task while being domain-invariant.

### Grasp Planning
Computing optimal gripper poses and approach trajectories for picking up objects, considering geometry and physics.

### Grasp Points
Locations on an object surface suitable for gripper contact, often computed from point cloud analysis.

### Grounding
Connecting abstract symbols (words, concepts) to physical world entities that the robot can perceive and act upon.

### Ground Truth
The known correct answer used to evaluate system performance. In simulation, can be obtained from the physics engine.

### Guardrail
A safety constraint that limits system behavior, preventing dangerous or unethical actions.

### Gyroscope
An inertial sensor measuring angular velocity. Combined with accelerometers in IMUs for orientation estimation.

---

## H

### Headless Mode
Running simulation without graphical display, enabling faster training and server deployment.

### Hierarchical Task Network (HTN)
A planning formalism that decomposes high-level tasks into subtasks recursively until primitive actions are reached.

### Hip Strategy
A balance recovery strategy using hip joint adjustments to shift the center of mass. Effective for medium perturbations.

### Homogeneous Transform
A 4×4 matrix representing 3D rotation and translation, enabling composition of transformations.

### Human-in-the-Loop
A system design where humans provide oversight, correction, or guidance during robot operation.

---

## I

### IK (Inverse Kinematics)
Computing joint angles that achieve a desired end-effector pose. Fundamental for manipulation and locomotion planning.

### IMU (Inertial Measurement Unit)
A sensor combining accelerometers and gyroscopes (and often magnetometers) to measure motion and orientation.

### Impedance Control
A control strategy that regulates the relationship between force and motion, making robots behave like spring-damper systems.

### Imitation Learning
Training policies from expert demonstrations rather than reward signals, enabling learning of complex behaviors.

### Inertia Tensor
A 3×3 matrix describing how mass is distributed in a rigid body, determining its rotational dynamics.

### Inflation Radius
The distance around obstacles in a costmap marked as high cost, preventing the robot from getting too close.

### Intent
The underlying goal of a user's command (e.g., "navigate", "manipulate", "query"). First step in natural language understanding.

### Intrinsic Calibration
Determining internal sensor parameters like focal length and distortion coefficients for cameras.

### Isaac Sim
NVIDIA's robotics simulation platform built on Omniverse, featuring photorealistic rendering and GPU-accelerated physics.

---

## J

### Jacobian
A matrix relating joint velocities to end-effector velocities. Essential for inverse kinematics and force control.

### JetPack
NVIDIA's SDK for Jetson platforms, including CUDA, cuDNN, TensorRT, and other AI/robotics libraries.

### Jitter
Variation in timing or latency. Network jitter affects remote control; control jitter affects motion smoothness.

### Joint Limits
The allowable range of motion for each robot joint, defined by mechanical constraints.

### Joint Space
The space of all possible joint configurations, as opposed to task space (end-effector poses).

---

## K

### Keyframe
A significant frame in a sequence used for SLAM, storing pose and visual features for loop closure and mapping.

### Kinematic Constraints
Restrictions on robot motion due to joint types, limits, and physical connections between links.

### Knowledge Base
A structured database of known objects, locations, and their properties for entity resolution and reasoning.

---

## L

### Latency
The time delay between input and output in a system. Critical for real-time robotics; typically measured in milliseconds.

### Latency Budget
The allocation of maximum allowable delays to each pipeline stage, ensuring end-to-end timing requirements are met.

### LiDAR
Light Detection and Ranging sensor that measures distances using laser pulses, producing 2D or 3D point clouds.

### Linear Algebra
Mathematical foundation for robotics, including vectors, matrices, transformations, and decompositions.

### LIPM (Linear Inverted Pendulum Model)
A simplified model of bipedal walking treating the robot as an inverted pendulum, enabling analytical gait planning.

### LLM (Large Language Model)
Neural networks trained on massive text corpora for language understanding and generation. Used for task planning and HRI.

### Localization
Determining the robot's position and orientation within a known map using sensor observations.

### Local Planner
A reactive controller that follows global paths while avoiding dynamic obstacles, typically running at high frequency.

### Log-Odds
A representation of probability as log(p/(1-p)), convenient for recursive Bayesian updates in occupancy grids.

### Loop Closure
Recognizing when the robot returns to a previously visited location, enabling map correction in SLAM.

---

## M

### Madgwick Filter
A computationally efficient sensor fusion algorithm for orientation estimation from IMU data.

### Manipulation
Robot interaction with objects: grasping, moving, placing, and tool use. A core capability for humanoid robots.

### Meta-Learning
Learning to learn; training models that can quickly adapt to new tasks with few examples.

### MMD (Maximum Mean Discrepancy)
A statistical measure of distribution difference used in domain adaptation to align source and target features.

### Monte Carlo Dropout
Using dropout at inference time to estimate prediction uncertainty through multiple stochastic forward passes.

### Motion Planning
Computing collision-free trajectories from start to goal configurations, considering robot kinematics and obstacles.

### MoveIt2
The ROS 2 motion planning framework providing IK, collision checking, and trajectory generation.

### MPC (Model Predictive Control)
An optimal control strategy that repeatedly solves a finite-horizon optimization problem, enabling constraint handling.

### Multimodal Fusion
Combining information from multiple sensor modalities (vision, audio, touch) for robust perception.

---

## N

### Nav2
The ROS 2 navigation stack providing path planning, obstacle avoidance, and localization for mobile robots.

### Noise Density
A parameter characterizing random noise in inertial sensors, measured in units like μg/√Hz or °/s/√Hz.

### Non-Maximum Suppression (NMS)
A post-processing algorithm eliminating redundant overlapping detections by keeping only highest-confidence predictions.

### Null-space
The set of joint velocities producing zero end-effector motion, enabling secondary objectives during manipulation.

### Numerical IK
Iterative algorithms (e.g., Newton-Raphson, Jacobian transpose) for solving inverse kinematics when analytical solutions don't exist.

---

## O

### Object Detection
Identifying and localizing objects in images or point clouds, outputting class labels and bounding boxes.

### Observation Space
The set of all possible observations an agent can receive in a reinforcement learning environment.

### Occupancy Grid
A 2D or 3D grid where each cell stores the probability of being occupied by an obstacle.

### Octree
A tree data structure for 3D spatial partitioning, efficiently representing sparse volumetric data.

### Omniverse
NVIDIA's platform for 3D design collaboration and simulation, underlying Isaac Sim.

### ONNX
Open Neural Network Exchange format enabling model portability between deep learning frameworks.

### Opset Version
ONNX operator set version defining available operations, important for compatibility with inference engines.

---

## P

### Parallel Environments
Running multiple simulation instances simultaneously for faster RL training through experience collection.

### Path Planning
Finding a collision-free path through configuration space, often using algorithms like A*, RRT, or PRM.

### PD Control
Proportional-Derivative control using position error and velocity error for smooth motion tracking.

### Perception
The robot's ability to sense and interpret its environment through cameras, LiDAR, IMUs, and other sensors.

### PhysX
NVIDIA's physics engine providing GPU-accelerated rigid body, articulation, and soft body simulation.

### Physical AI
AI systems that interact with the physical world through embodied robots, integrating perception, planning, and control.

### Pipeline Architecture
Organizing processing as a sequence of stages, enabling parallelism and modular design.

### Point Cloud
A set of 3D points representing surface geometry, typically from LiDAR or depth cameras.

### Policy
A mapping from states to actions in reinforcement learning, representing the agent's decision-making strategy.

### PPO (Proximal Policy Optimization)
A popular RL algorithm using clipped surrogate objectives for stable policy updates. Widely used for locomotion.

### Preemption
Interrupting an ongoing action to handle a higher-priority request, common in action server implementations.

### Pre-trained Model
A model trained on a large dataset that can be fine-tuned for specific tasks, enabling transfer learning.

### Preview Control
A control strategy using future reference trajectory information to improve tracking performance.

### Proprioception
Sensing internal state (joint positions, velocities, torques) as opposed to external perception.

### Prosody
The rhythm, stress, and intonation of speech, important for natural-sounding TTS.

### Publisher
A ROS 2 node component that sends messages on a topic for subscribers to receive.

---

## Q

### QoS (Quality of Service)
DDS settings controlling message delivery guarantees, including reliability, durability, and deadline.

### Quantization
Reducing model precision (FP32 → FP16 → INT8) to decrease memory usage and increase inference speed.

### Quaternion
A four-element representation of 3D rotation avoiding gimbal lock, expressed as (w, x, y, z).

---

## R

### Random Walk
A noise process where each sample adds to the previous, causing unbounded drift. Affects IMU integration.

### RANSAC
Random Sample Consensus algorithm for robust model fitting despite outliers, used in plane detection and localization.

### Ray Casting
Computing ray-surface intersections for collision detection, sensor simulation, or visibility queries.

### Reachability
The set of poses the robot's end-effector can achieve, determined by kinematics and joint limits.

### Real-Time
Systems where correctness depends on timely completion, not just logical correctness. Critical for robot control.

### Reality Gap
The discrepancy between simulation and real-world behavior that can cause policies trained in simulation to fail.

### Recovery Behavior
Actions taken when primary behavior fails, such as backing up when navigation is stuck.

### Reinforcement Learning (RL)
Learning through trial and error by maximizing cumulative reward, enabling complex behavior acquisition.

### Reliability
A DDS QoS setting determining whether messages are guaranteed delivered (reliable) or best-effort.

### Replanning
Generating a new plan when the current plan becomes invalid due to changed conditions or failures.

### Reprojection Error
The pixel distance between observed and predicted feature locations, used to evaluate calibration accuracy.

### Reward Shaping
Designing reward functions to guide RL training toward desired behaviors, critical for sample efficiency.

### RGB-D Camera
A camera providing both color (RGB) and depth (D) images, enabling 3D perception.

### Rigid Body
A solid object that maintains its shape under forces, the fundamental entity in physics simulation.

### ROS 2
Robot Operating System 2, the open-source robotics middleware providing communication, tools, and libraries.

### RRT (Rapidly-exploring Random Tree)
A sampling-based motion planning algorithm that incrementally builds a tree exploring configuration space.

### RTAB-Map
Real-Time Appearance-Based Mapping, a RGB-D SLAM system for 3D mapping and localization.

---

## S

### SAC (Soft Actor-Critic)
An off-policy RL algorithm maximizing both reward and entropy, providing good exploration and sample efficiency.

### Safety Constraint
A hard limit on robot behavior (e.g., force limits, forbidden zones) that must never be violated.

### Sample Rate
The frequency of sensor measurements, affecting temporal resolution and aliasing.

### Scene Graph
A hierarchical representation of objects and their relationships in an environment.

### SDF (Simulation Description Format)
An XML format for describing robots and environments in Gazebo simulation.

### Self-Collision
Contact between different parts of the same robot, which must be avoided during motion planning.

### Semantic Segmentation
Classifying each pixel in an image by object category, enabling detailed scene understanding.

### Sensor Fusion
Combining data from multiple sensors to achieve better estimates than any single sensor alone.

### Sequence Node
A Behavior Tree node that executes children in order, succeeding only if all children succeed.

### Service
A ROS 2 synchronous communication pattern for request-response interactions.

### Sim-to-Real
Transferring models or policies trained in simulation to work on physical robots.

### Simulation Timestep
The time increment for physics updates, affecting accuracy and stability (typically 1-10ms).

### SLAM (Simultaneous Localization and Mapping)
Building a map while simultaneously tracking the robot's position within it.

### Slip
Relative motion between contacting surfaces, important for grasp stability and locomotion traction.

### Smith Predictor
A control technique for systems with significant time delay, using a model to predict future states.

### Solver Iterations
The number of constraint solving passes per physics step, trading accuracy for performance.

### Sparse Map
A SLAM map storing only distinctive visual features rather than dense geometry.

### Spatial Grounding
Connecting language references like "the cup on the table" to physical locations and objects.

### SRDF (Semantic Robot Description Format)
An XML file defining semantic information about a robot: planning groups, collision pairs, and poses.

### Stability
The property of a system returning to equilibrium after perturbation. Critical for balance control.

### State Space
The set of all possible states a system can occupy, forming the domain for control and planning.

### Stereo Depth
Estimating depth from the disparity between two camera views, mimicking human binocular vision.

### Stepping Strategy
A balance recovery strategy where the humanoid takes a step to enlarge the support polygon for large disturbances.

### Subscriber
A ROS 2 node component that receives messages published on a topic.

### Support Polygon
The convex hull of ground contact points; the center of mass must stay above it for static stability.

### Synonym Mapping
Converting equivalent words to canonical forms (e.g., "grab" → "pick") for robust language understanding.

---

## T

### Task Decomposition
Breaking complex tasks into simpler subtasks that can be executed sequentially or in parallel.

### TensorRT
NVIDIA's inference optimizer and runtime for deploying deep learning models with maximum performance on GPUs.

### TF2
The ROS 2 transform library managing coordinate frame relationships over time.

### Timestep
The discrete time interval between simulation or control updates.

### Topic
A named channel in ROS 2 for publish-subscribe communication between nodes.

### TracIK
A fast IK solver combining KDL and SQP methods for robust inverse kinematics solutions.

### Trajectory
A time-parameterized path specifying positions, velocities, and accelerations over time.

### Transfer Learning
Using knowledge from one task or domain to improve learning on a different but related task.

### Transform
A representation of position and orientation between coordinate frames, typically as translation + rotation.

### Transient Local
A DDS durability setting where messages are kept for late-joining subscribers within the same node lifecycle.

### Turn-Taking
The protocol for managing speaker changes in dialogue, important for natural conversation flow.

---

## U

### URDF (Unified Robot Description Format)
An XML format describing robot kinematics, dynamics, and visual/collision geometry for ROS.

### Uncertainty Quantification
Estimating the confidence or reliability of model predictions, critical for safe robot decision-making.

---

## V

### Vectorized Environment
Multiple parallel environment instances sharing computation for efficient RL training.

### Velocity Control
Controlling joint or end-effector velocities rather than positions, useful for compliant motion.

### Vision-Language Model (VLM)
Models that understand both images and text, enabling visual question answering and image-based reasoning.

### Visual SLAM
SLAM using camera images rather than LiDAR, relying on visual features for localization and mapping.

### Voxel
A volumetric pixel; the 3D equivalent of a 2D pixel, used in occupancy grids and point cloud processing.

### Voxel Grid
A 3D grid of voxels for representing volumetric data or downsampling point clouds.

### VRAM
Video RAM on GPUs, limiting model size and batch sizes for deep learning inference.

---

## W

### Warmup Iterations
Initial simulation steps to let physics settle before measurement or training begins.

### Waypoint
An intermediate point along a path that the robot should pass through.

### Whisper
OpenAI's speech recognition model providing accurate transcription with multilingual support.

### Whole-Body Control
Coordinated control of all robot joints simultaneously, enabling complex motions like walking while manipulating.

### Working Memory
Short-term storage of current context and goals during task execution.

### World Model
An internal representation of environment dynamics that can be used for planning and prediction.

### Workspace
The volume of space reachable by the robot's end-effector.

---

## X-Z

### XACRO
XML Macros for ROS, enabling parameterized and modular URDF files.

### YAML
A human-readable data serialization format used for ROS configuration files.

### Zero-Shot
Performing a task without any task-specific training examples, relying on general knowledge or language descriptions.

### ZMP (Zero Moment Point)
The point on the ground where the total moment of inertial and gravity forces is zero; key for biped balance.

---

## Acronym Reference Table

| Acronym | Full Form |
|---------|-----------|
| AHRS | Attitude and Heading Reference System |
| API | Application Programming Interface |
| CNN | Convolutional Neural Network |
| CoM | Center of Mass |
| CUDA | Compute Unified Device Architecture |
| DDS | Data Distribution Service |
| DOF | Degrees of Freedom |
| EKF | Extended Kalman Filter |
| FCL | Flexible Collision Library |
| FPS | Frames Per Second |
| GPU | Graphics Processing Unit |
| HRI | Human-Robot Interaction |
| HTN | Hierarchical Task Network |
| IK | Inverse Kinematics |
| IMU | Inertial Measurement Unit |
| LiDAR | Light Detection and Ranging |
| LIPM | Linear Inverted Pendulum Model |
| LLM | Large Language Model |
| MPC | Model Predictive Control |
| NLP | Natural Language Processing |
| NMS | Non-Maximum Suppression |
| ONNX | Open Neural Network Exchange |
| PD | Proportional-Derivative |
| PPO | Proximal Policy Optimization |
| QoS | Quality of Service |
| RANSAC | Random Sample Consensus |
| RGB-D | Red-Green-Blue-Depth |
| RL | Reinforcement Learning |
| ROS | Robot Operating System |
| RRT | Rapidly-exploring Random Tree |
| SAC | Soft Actor-Critic |
| SDF | Simulation Description Format |
| SLAM | Simultaneous Localization and Mapping |
| SRDF | Semantic Robot Description Format |
| TF | Transform |
| ToF | Time of Flight |
| TTS | Text-to-Speech |
| URDF | Unified Robot Description Format |
| VLM | Vision-Language Model |
| VRAM | Video Random Access Memory |
| ZMP | Zero Moment Point |

---

:::info Connection to Capstone
This glossary supports all modules of the textbook. When encountering unfamiliar terms in any section, refer here for definitions. The capstone project integrates concepts from across this entire glossary.
:::

---

## Further Reading

For deeper understanding of these concepts, refer to the relevant sections:

- **Perception terms**: Module 2 (Simulation) and Module 3 (Perception)
- **Control terms**: Module 3 (Locomotion Control)
- **Planning terms**: Module 4 (System Integration)
- **Hardware terms**: Module 1 (Foundations)
- **AI/ML terms**: Throughout all modules

