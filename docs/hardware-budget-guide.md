---
id: hardware-budget-guide
title: Hardware Budget Guide
sidebar_position: 102
keywords: ['hardware', 'budget', 'purchasing', 'equipment', 'cost']
---

# 💰 Hardware Budget Guide

This guide provides hardware recommendations across three budget tiers for building Physical AI and humanoid robotics systems. Each tier enables progressively more advanced capabilities.

---

## Budget Tier Overview

| Tier | Budget | Best For | Key Capabilities |
|------|--------|----------|------------------|
| **Starter** | $500 - $1,500 | Students, Hobbyists | Simulation, basic perception, learning fundamentals |
| **Professional** | $2,000 - $5,000 | Engineers, Researchers | Real hardware integration, edge AI, development |
| **Production** | $10,000 - $25,000+ | Labs, Companies | Full humanoid systems, high-performance compute |

---

## 🟢 Starter Tier ($500 - $1,500)

### Recommended for:
- Students learning robotics fundamentals
- Hobbyists exploring Physical AI concepts
- Following along with simulation-based chapters
- Building portfolio projects

### Core Setup

| Component | Recommendation | Price (USD) | Notes |
|-----------|---------------|-------------|-------|
| **Computer** | Gaming laptop with RTX 3060+ | $800-1,200 | 16GB RAM minimum, Linux dual-boot |
| **GPU** | NVIDIA RTX 3060 (built-in) | Included | 12GB VRAM sufficient for Isaac Sim |
| **Camera** | Intel RealSense D435i | $300 | RGB-D for perception projects |
| **Microphone** | ReSpeaker USB Array | $80 | 4-mic array for voice projects |

### Alternative Budget Options

| Component | Budget Option | Price (USD) |
|-----------|--------------|-------------|
| **Camera** | OAK-D Lite | $150 |
| **Camera** | ZED Mini (used) | $200 |
| **Computer** | Refurbished workstation + RTX 3060 | $600-800 |

### What You Can Build

- ✅ Complete all simulation chapters (M2)
- ✅ Basic object detection and perception
- ✅ Voice interaction prototypes
- ✅ Reinforcement learning experiments
- ⚠️ Limited real robot integration
- ❌ Full humanoid development

### Sample Configuration: $1,200

```
Starter Kit:
├── Used gaming laptop (RTX 3060, 16GB) ... $800
├── Intel RealSense D435i ................ $300
├── ReSpeaker USB Mic Array .............. $80
└── USB hub + cables ..................... $20
                                    Total: $1,200
```

---

## 🟡 Professional Tier ($2,000 - $5,000)

### Recommended for:
- Robotics engineers and researchers
- Startup prototyping
- University research labs
- Serious development work

### Core Setup

| Component | Recommendation | Price (USD) | Notes |
|-----------|---------------|-------------|-------|
| **Workstation GPU** | NVIDIA RTX 4080/4090 | $1,200-2,000 | 16-24GB VRAM for large models |
| **Workstation** | Custom build or Dell Precision | $2,000-3,000 | 64GB RAM, NVMe SSD |
| **Edge Device** | NVIDIA Jetson Orin Nano | $500 | 8GB for deployment |
| **Depth Camera** | Intel RealSense D455 | $350 | Longer range than D435i |
| **Robot Arm** | Interbotix WidowX 250 | $2,500 | 6-DOF research arm |
| **Gripper** | Robotiq 2F-85 (used) | $800-1,200 | Or 3D printed alternative |

### Compute Options

| Setup | Components | Price | Use Case |
|-------|-----------|-------|----------|
| **Desktop + Jetson** | RTX 4080 + Orin Nano | $2,700 | Training + edge deployment |
| **Dual GPU** | 2x RTX 4080 | $2,400 | Faster training, parallel sim |
| **Cloud Hybrid** | RTX 3070 + Cloud credits | $1,500 | Burst training capacity |

### Sensor Suite

| Sensor | Model | Price | Purpose |
|--------|-------|-------|---------|
| **RGB-D Camera** | RealSense D455 | $350 | Manipulation perception |
| **LiDAR** | RPLidar A1 | $100 | 2D navigation |
| **LiDAR** | Ouster OS0-32 | $2,500 | 3D perception (optional) |
| **IMU** | Microstrain 3DM-CV7 | $400 | High-quality inertial |
| **Microphone** | ReSpeaker Core v2 | $150 | Advanced voice processing |
| **Speakers** | Powered monitors | $100 | TTS output |

### What You Can Build

- ✅ All textbook content including hardware integration
- ✅ Arm manipulation with real robot
- ✅ Edge AI deployment on Jetson
- ✅ Multi-sensor fusion systems
- ✅ Research-quality experiments
- ⚠️ Limited mobile base capabilities
- ❌ Full humanoid system

### Sample Configuration: $4,500

```
Professional Kit:
├── Custom Workstation
│   ├── AMD Ryzen 9 7900X ................ $450
│   ├── NVIDIA RTX 4080 16GB ............. $1,200
│   ├── 64GB DDR5 RAM .................... $200
│   ├── 2TB NVMe SSD ..................... $150
│   ├── Case + PSU + Motherboard ......... $400
│                              Subtotal: $2,400
├── NVIDIA Jetson Orin Nano Dev Kit ...... $500
├── Intel RealSense D455 ................. $350
├── ReSpeaker Mic Array v2.0 ............. $80
├── Interbotix WidowX 250 6DOF ........... $900 (edu discount)
├── 3D Printed Gripper + Filament ........ $150
└── Cables, mounts, accessories .......... $120
                                    Total: $4,500
```

---

## 🔴 Production Tier ($10,000 - $25,000+)

### Recommended for:
- Research institutions
- Robotics companies
- Advanced R&D labs
- Production deployments

### Core Setup

| Component | Recommendation | Price (USD) | Notes |
|-----------|---------------|-------------|-------|
| **Training Server** | Multi-GPU workstation | $8,000-15,000 | 2-4x RTX 4090 or A6000 |
| **Edge Compute** | Jetson AGX Orin 64GB | $2,000 | Maximum edge performance |
| **Robot Platform** | Research humanoid | $15,000-100,000+ | Unitree H1, etc. |
| **Sensor Suite** | Multi-camera, LiDAR | $5,000-10,000 | Comprehensive perception |

### Compute Infrastructure

| Configuration | Specs | Price | Performance |
|--------------|-------|-------|-------------|
| **Entry Server** | 2x RTX 4090, 128GB | $8,000 | Good for most research |
| **Research Server** | 4x RTX 4090, 256GB | $15,000 | Large model training |
| **Enterprise** | 4x A6000, 512GB | $30,000 | 48GB VRAM per GPU |
| **Cloud Equivalent** | AWS p4d.24xlarge | $32/hour | 8x A100 on demand |

### Robot Platforms by Category

#### Humanoid Robots

| Robot | Price | Capabilities | Availability |
|-------|-------|-------------|--------------|
| **Unitree H1** | $90,000 | Full humanoid, 47 DOF | Available |
| **Unitree G1** | $16,000 | Compact humanoid | Pre-order |
| **1X NEO** | TBD | Consumer humanoid | Development |
| **Open Source** | $5,000-20,000 | Custom build | DIY |

#### Research Arms

| Arm | Price | Payload | Use Case |
|-----|-------|---------|----------|
| **Franka Emika Panda** | $20,000 | 3kg | Research standard |
| **Kinova Gen3** | $25,000 | 4kg | Collaborative |
| **Universal Robots UR5e** | $35,000 | 5kg | Industrial |
| **Interbotix ViperX 300** | $5,000 | 750g | Budget research |

#### Mobile Bases

| Platform | Price | Type | Notes |
|----------|-------|------|-------|
| **Clearpath Jackal** | $20,000 | UGV | Outdoor capable |
| **Turtlebot 4** | $1,200 | Indoor | ROS 2 native |
| **Unitree Go2** | $1,600 | Quadruped | Locomotion research |

### Production Sensor Suite

| Sensor | Model | Price | Specs |
|--------|-------|-------|-------|
| **3D Camera** | Photoneo MotionCam-3D | $5,000 | Industrial grade |
| **RGB Camera** | FLIR Blackfly S | $800 | Global shutter |
| **LiDAR** | Ouster OS1-64 | $8,000 | 64-channel 3D |
| **Force/Torque** | ATI Gamma | $6,000 | 6-axis F/T |
| **Tactile** | DIGIT sensor | $300 | Vision-based touch |

### What You Can Build

- ✅ Full humanoid systems
- ✅ Production deployments
- ✅ Multi-robot coordination
- ✅ Advanced research experiments
- ✅ Commercial applications
- ✅ Everything in this textbook

### Sample Configuration: $25,000

```
Production Research Kit:
├── Training Workstation
│   ├── AMD Threadripper 7960X ........... $1,500
│   ├── 2x NVIDIA RTX 4090 24GB .......... $4,000
│   ├── 256GB DDR5 ECC RAM ............... $1,000
│   ├── 4TB NVMe RAID .................... $600
│   ├── Server case + 1600W PSU .......... $500
│                              Subtotal: $7,600
├── Edge Deployment
│   ├── Jetson AGX Orin 64GB ............. $2,000
│   ├── Carrier board + enclosure ........ $500
│                              Subtotal: $2,500
├── Manipulation Platform
│   ├── Interbotix ViperX 300 6DOF ....... $5,000
│   ├── Robotiq 2F-140 Gripper ........... $3,000
│   ├── ATI Nano17 F/T Sensor ............ $2,500
│                              Subtotal: $10,500
├── Sensor Suite
│   ├── 2x Intel RealSense D455 .......... $700
│   ├── Ouster OS0-32 LiDAR .............. $2,500
│   ├── ReSpeaker Core v2 ................ $150
│   ├── FLIR Blackfly S camera ........... $800
│                              Subtotal: $4,150
└── Infrastructure
    ├── Networking (switch, cables) ...... $200
    └── Mounts, brackets, misc ........... $250
                               Subtotal: $450
                                  Total: $25,200
```

---

## Component Deep Dive

### GPU Selection Guide

| GPU | VRAM | Price | Best For |
|-----|------|-------|----------|
| RTX 3060 | 12GB | $300 | Learning, light simulation |
| RTX 4070 Ti | 12GB | $700 | Development, medium models |
| RTX 4080 | 16GB | $1,200 | Research, Isaac Sim |
| RTX 4090 | 24GB | $2,000 | Training, large models |
| RTX A5000 | 24GB | $2,500 | Professional, ECC memory |
| RTX A6000 | 48GB | $5,000 | Large models, enterprise |
| H100 | 80GB | $30,000 | Maximum performance |

:::tip GPU Memory Requirements
- **Isaac Sim**: 8GB minimum, 16GB+ recommended
- **YOLOv8**: 4GB minimum
- **LLM inference (7B)**: 8GB minimum
- **LLM inference (70B)**: 48GB+ or quantized
- **Diffusion Policy training**: 12GB+ recommended
:::

### Edge Computing Options

| Device | GPU | CPU | RAM | Power | Price |
|--------|-----|-----|-----|-------|-------|
| Jetson Nano | 128-core Maxwell | 4-core A57 | 4GB | 10W | $150 |
| Jetson Orin Nano | 1024-core Ampere | 6-core A78 | 8GB | 15W | $500 |
| Jetson AGX Orin | 2048-core Ampere | 12-core A78 | 32-64GB | 60W | $2,000 |

### Camera Comparison

| Camera | Resolution | Depth Range | FPS | Interface | Price |
|--------|-----------|-------------|-----|-----------|-------|
| RealSense D435i | 1920x1080 | 0.2-10m | 90 | USB 3 | $300 |
| RealSense D455 | 1280x800 | 0.4-20m | 90 | USB 3 | $350 |
| ZED 2i | 2208x1242 | 0.3-20m | 100 | USB 3 | $450 |
| OAK-D Pro | 1280x800 | 0.2-35m | 120 | USB 3 | $300 |
| Azure Kinect | 3840x2160 | 0.5-5m | 30 | USB 3 | $400 |

---

## Cost Optimization Strategies

### 1. Educational Discounts

| Vendor | Discount | Eligibility |
|--------|----------|-------------|
| NVIDIA | 10-20% | Academic institutions |
| Interbotix | 15% | Students, educators |
| Universal Robots | Academic pricing | Universities |
| Intel | Developer program | Registered developers |

### 2. Used/Refurbished Options

| Component | New Price | Used Price | Where to Buy |
|-----------|-----------|------------|--------------|
| RTX 3080 | $700 | $400 | eBay, r/hardwareswap |
| RealSense D435 | $300 | $150 | eBay |
| Jetson Xavier | $1,000 | $400 | eBay |
| UR5 arm | $35,000 | $15,000 | Surplus auctions |

### 3. Cloud Computing for Training

| Provider | Instance | Price/Hour | Best For |
|----------|----------|------------|----------|
| Lambda Labs | A100 80GB | $1.29 | RL training |
| AWS | p4d.24xlarge | $32.77 | Large scale |
| Google Cloud | A100 40GB | $2.93 | General ML |
| RunPod | RTX 4090 | $0.44 | Budget training |

### 4. DIY and Open Source

| Component | Commercial | DIY Alternative | Savings |
|-----------|-----------|----------------|---------|
| Gripper | $1,500+ | 3D printed + servos | $1,400 |
| Mobile base | $20,000 | Custom build | $18,000 |
| Arm | $5,000+ | Actuator + 3D print | $4,000 |
| Enclosure | $500 | Aluminum extrusion | $400 |

---

## Buying Checklist

### Before Purchasing

- [ ] Verify GPU VRAM meets requirements for your use case
- [ ] Check power supply capacity for GPU(s)
- [ ] Ensure Ubuntu/Linux compatibility
- [ ] Verify ROS 2 Humble support for sensors
- [ ] Check return policy and warranty
- [ ] Research driver compatibility

### After Purchasing

- [ ] Install Ubuntu 22.04 LTS
- [ ] Install NVIDIA drivers and CUDA
- [ ] Set up ROS 2 Humble
- [ ] Install Isaac Sim (if applicable)
- [ ] Verify all sensors work with ROS 2
- [ ] Run benchmark tests
- [ ] Create system backup

---

## Vendor Directory

### Compute Hardware

| Vendor | Products | Website |
|--------|----------|---------|
| NVIDIA | GPUs, Jetson | nvidia.com |
| AMD | CPUs, GPUs | amd.com |
| Lambda Labs | Workstations | lambdalabs.com |
| Puget Systems | Custom builds | pugetsystems.com |

### Sensors

| Vendor | Products | Website |
|--------|----------|---------|
| Intel RealSense | RGB-D cameras | intelrealsense.com |
| Stereolabs | ZED cameras | stereolabs.com |
| Ouster | LiDAR | ouster.com |
| Luxonis | OAK cameras | luxonis.com |
| Seeed Studio | ReSpeaker mics | seeedstudio.com |

### Robot Platforms

| Vendor | Products | Website |
|--------|----------|---------|
| Interbotix | Research arms | trossenrobotics.com |
| Universal Robots | Collaborative arms | universal-robots.com |
| Unitree | Quadrupeds, humanoids | unitree.com |
| Clearpath | Mobile robots | clearpathrobotics.com |
| Robotiq | Grippers | robotiq.com |

---

## Quick Recommendations by Use Case

| Goal | Tier | Key Components | Budget |
|------|------|----------------|--------|
| Learn simulation | Starter | Laptop + RealSense | $1,200 |
| Capstone project | Professional | Workstation + Jetson + Arm | $4,500 |
| Research lab | Production | Multi-GPU + Full sensor suite | $25,000 |
| Startup MVP | Professional | Workstation + Robot arm | $8,000 |
| Full humanoid R&D | Production | Server + Unitree H1 | $100,000+ |

---

:::info Connection to Textbook
This guide supports the hardware requirements mentioned throughout the textbook:
- **Module 1**: Workstation and edge device setup
- **Module 2**: Simulation requires GPU with 8GB+ VRAM
- **Module 3**: Perception requires cameras and compute
- **Module 4**: Integration benefits from full sensor suite
:::

