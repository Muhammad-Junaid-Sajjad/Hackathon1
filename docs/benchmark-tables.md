---
id: benchmark-tables
title: Benchmark Comparison Tables
sidebar_position: 105
keywords: ['benchmark', 'comparison', 'performance', 'metrics', 'evaluation', 'yolo12', 'yolo26', '2025']
last_updated: 2025-12-29
---

# 📊 Benchmark Comparison Tables

:::info 📅 Updated December 29, 2025
Benchmarks updated to include **YOLO12** (Feb 2025), **YOLO26** (Sep 2025), **GR00T N1.6**, and **Jetson Thor** performance metrics.
:::

This section provides benchmark comparisons for algorithms, models, and systems covered in the textbook against published state-of-the-art results.

---

## Object Detection Benchmarks

### COCO Dataset Performance (December 2025)

Comparison of object detection models for robotics applications:

| Model | Backbone | mAP@50 | mAP@50:95 | FPS (RTX 4090) | FPS (Jetson Thor) | Parameters |
|-------|----------|--------|-----------|----------------|-------------------|------------|
| [YOLO26n](https://arxiv.org/abs/2510.09653) ⭐ | NMS-free | 39.8 | 24.1 | 850 | 420 | 2.4M |
| [YOLO26s](https://arxiv.org/abs/2510.09653) ⭐ | NMS-free | 47.5 | 31.0 | 680 | 340 | 8.9M |
| [YOLO12n](https://docs.ultralytics.com/models/yolo12/) | Area Attention | 40.6 | 24.5 | 750 | 380 | 2.5M |
| [YOLO12s](https://docs.ultralytics.com/models/yolo12/) | Area Attention | 48.0 | 31.5 | 580 | 290 | 9.2M |
| [YOLO12m](https://docs.ultralytics.com/models/yolo12/) | R-ELAN | 52.5 | 35.8 | 380 | 185 | 19.8M |
| [YOLOv11n](https://docs.ultralytics.com/models/yolo11/) | C3K2 | 39.5 | 23.8 | 710 | 350 | 2.6M |
| [YOLOv11s](https://docs.ultralytics.com/models/yolo11/) | C3K2 | 47.0 | 30.2 | 520 | 260 | 9.4M |
| [YOLOv11m](https://docs.ultralytics.com/models/yolo11/) | C3K2 | 51.5 | 34.8 | 320 | 160 | 20.1M |
| [RT-DETR-L](https://github.com/lyuwenyu/RT-DETR) | ResNet-50 | 53.0 | 36.1 | 114 | 55 | 32M |
| [RT-DETR-X](https://github.com/lyuwenyu/RT-DETR) | ResNet-101 | 54.8 | 37.6 | 74 | 35 | 67M |

⭐ = **2025 State-of-the-Art** | Jetson Thor benchmarks use Blackwell GPU (2,070 TFLOPS)

:::tip For Real-Time Robotics (December 2025)
**Recommended Models:**
- **[YOLO26s](https://arxiv.org/abs/2510.09653)** (Sep 2025): NMS-free end-to-end detection, fastest inference, best for latency-critical applications
- **[YOLO12s](https://docs.ultralytics.com/models/yolo12/)** (Feb 2025): Attention-centric architecture, 2.1% better mAP than YOLOv11
- **[YOLOv11m](https://docs.ultralytics.com/models/yolo11/)**: Most stable for production, extensive ecosystem support

**Note:** YOLO26 eliminates NMS post-processing entirely, reducing latency by ~15% vs YOLO12.
:::

### Instance Segmentation (COCO)

| Model | mAP (box) | mAP (mask) | FPS (RTX 4090) | FPS (Jetson Orin) |
|-------|-----------|------------|----------------|-------------------|
| [YOLOv11n-seg](https://docs.ultralytics.com/models/yolo11/) | 38.9 | 32.0 | 610 | 165 |
| [YOLOv11s-seg](https://docs.ultralytics.com/models/yolo11/) | 46.6 | 38.8 | 450 | 110 |
| [YOLOv11m-seg](https://docs.ultralytics.com/models/yolo11/) | 51.5 | 42.5 | 265 | 58 |
| [Mask R-CNN](https://github.com/facebookresearch/detectron2) | 38.2 | 34.7 | 32 | 5 |
| [SAM 2](https://github.com/facebookresearch/sam2) (Hiera-B+) | - | 64.2* | 25 | 4 |
| [FastSAM](https://github.com/CASIA-IVA-Lab/FastSAM) | - | 58.3* | 85 | 18 |

*SAM metrics use different evaluation; not directly comparable. [SAM 2](https://ai.meta.com/sam2/) is Meta's latest segmentation model with video support.

---

## Depth Estimation Benchmarks

### NYU Depth v2 Dataset

| Model | RMSE (m) | AbsRel | δ < 1.25 | FPS (RTX 4090) | FPS (Jetson) |
|-------|----------|--------|----------|----------------|--------------|
| MiDaS v3.1 (Small) | 0.364 | 0.098 | 0.912 | 180 | 45 |
| MiDaS v3.1 (Large) | 0.312 | 0.082 | 0.938 | 65 | 12 |
| DPT-Large | 0.298 | 0.076 | 0.945 | 48 | 8 |
| ZoeDepth (NYU) | 0.270 | 0.068 | 0.955 | 35 | 6 |
| Metric3D v2 | 0.248 | 0.059 | 0.968 | 25 | 4 |

:::note Depth for Manipulation
For manipulation tasks where metric accuracy matters, use RealSense depth directly or ZoeDepth. For relative depth (obstacle avoidance), MiDaS Small offers best speed/accuracy tradeoff.
:::

---

## Speech Recognition Benchmarks

### LibriSpeech Dataset

Comparison of ASR models discussed in M4-C1-S2:

| Model | WER (clean) | WER (other) | RTF (CPU) | RTF (GPU) | Model Size |
|-------|-------------|-------------|-----------|-----------|------------|
| Whisper tiny | 7.6% | 14.9% | 0.8x | 0.05x | 39M |
| Whisper base | 5.0% | 10.8% | 1.2x | 0.08x | 74M |
| Whisper small | 3.4% | 7.6% | 2.5x | 0.15x | 244M |
| Whisper medium | 2.9% | 6.3% | 5.0x | 0.30x | 769M |
| Whisper large-v3 | 2.5% | 5.2% | 12.0x | 0.70x | 1.55B |
| Faster-Whisper (large) | 2.5% | 5.2% | 4.0x | 0.25x | 1.55B |
| Distil-Whisper | 3.0% | 6.5% | 2.0x | 0.12x | 756M |

*RTF = Real-Time Factor (lower is faster); 1.0x means real-time

:::tip For Robot Voice Interface
**Recommended:** Whisper small or Faster-Whisper medium for on-device deployment. Use Whisper large-v3 for highest accuracy when latency permits.
:::

---

## Motion Planning Benchmarks

### MoveIt2 Planner Comparison (Panda Arm)

| Planner | Success Rate | Planning Time (ms) | Path Length | Smoothness |
|---------|-------------|-------------------|-------------|------------|
| OMPL RRTConnect | 98.5% | 45 ± 32 | 1.15x optimal | Medium |
| OMPL RRT* | 95.2% | 890 ± 450 | 1.02x optimal | High |
| OMPL PRM | 94.8% | 120 ± 85 | 1.08x optimal | Medium |
| OMPL BiTRRT | 96.3% | 68 ± 48 | 1.12x optimal | Medium |
| STOMP | 92.1% | 250 ± 120 | 1.05x optimal | Very High |
| CHOMP | 89.5% | 180 ± 95 | 1.03x optimal | Very High |
| Pilz Industrial | 99.8% | 15 ± 5 | 1.25x optimal | Low |

### IK Solver Performance

| Solver | Success Rate | Time (μs) | Notes |
|--------|-------------|-----------|-------|
| KDL | 92% | 850 | Default, analytical+numerical |
| TracIK | 98% | 420 | Recommended, combines methods |
| IKFast | 99.5% | 12 | Pre-computed, robot-specific |
| BioIK | 97% | 650 | Evolutionary, handles constraints |
| Relaxed IK | 94% | 380 | Optimization-based |

---

## Locomotion Benchmarks

### Bipedal Walking (Isaac Gym)

| Algorithm | Walking Speed | Energy Efficiency | Fall Rate | Training Time |
|-----------|--------------|-------------------|-----------|---------------|
| PPO (baseline) | 1.2 m/s | 0.65 CoT | 8% | 4 hours |
| SAC | 1.1 m/s | 0.70 CoT | 12% | 6 hours |
| PPO + Curriculum | 1.5 m/s | 0.58 CoT | 3% | 8 hours |
| AMP (motion prior) | 1.4 m/s | 0.52 CoT | 2% | 12 hours |
| Diffusion Policy | 1.3 m/s | 0.55 CoT | 4% | 10 hours |

*CoT = Cost of Transport (lower is more efficient)

### Balance Recovery Performance

| Strategy | Perturbation Tolerance | Response Time | Recovery Success |
|----------|----------------------|---------------|-----------------|
| Ankle Strategy | ±5 cm CoM shift | 150ms | 98% |
| Hip Strategy | ±10 cm CoM shift | 200ms | 95% |
| Stepping Strategy | ±20 cm CoM shift | 350ms | 88% |
| Combined (ML) | ±25 cm CoM shift | 180ms | 94% |

---

## Grasp Detection Benchmarks

### GraspNet-1Billion Dataset

| Method | AP | AP (seen) | AP (novel) | FPS |
|--------|-----|----------|------------|-----|
| GraspNet Baseline | 27.5 | 33.4 | 16.3 | 8 |
| GPD | 31.2 | 38.1 | 18.5 | 12 |
| PointNetGPD | 38.4 | 45.2 | 25.8 | 25 |
| Contact-GraspNet | 52.3 | 61.2 | 34.5 | 15 |
| AnyGrasp | 58.7 | 67.3 | 41.2 | 20 |
| GraspGPT | 61.2 | 69.8 | 44.5 | 8 |

### Real Robot Grasp Success Rates

| Method | YCB Objects | Novel Objects | Cluttered Bin |
|--------|-------------|---------------|---------------|
| Heuristic | 72% | 45% | 38% |
| Dex-Net 2.0 | 85% | 68% | 55% |
| 6-DOF GraspNet | 89% | 75% | 62% |
| Contact-GraspNet | 92% | 82% | 71% |
| Human Teleop | 98% | 96% | 89% |

---

## SLAM Benchmarks

### TUM RGB-D Dataset

| Algorithm | ATE RMSE (m) | RPE (m/s) | FPS | Loop Closure |
|-----------|-------------|-----------|-----|--------------|
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | 0.010 | 0.008 | 30 | Yes |
| [RTAB-Map](http://introlab.github.io/rtabmap/) | 0.012 | 0.010 | 25 | Yes |
| [ElasticFusion](https://github.com/mp3guy/ElasticFusion) | 0.015 | 0.012 | 20 | Yes |
| [BadSLAM](https://github.com/ETH3D/badslam) | 0.009 | 0.007 | 15 | Yes |
| [DynaSLAM](https://github.com/BertaBesworked/DynaSLAM) | 0.008 | 0.006 | 8 | Yes |
| [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM) | 0.006 | 0.005 | 5 | Yes |
| [Gaussian Splatting SLAM](https://github.com/muskie82/MonoGS) | 0.007 | 0.006 | 12 | Yes |
| [cuVSLAM](https://developer.nvidia.com/isaac/cuvslam) (NVIDIA) | 0.009 | 0.007 | 60+ | Yes |

*ATE = Absolute Trajectory Error; RPE = Relative Pose Error

:::tip NVIDIA Isaac Integration
For production robotics on Jetson platforms, [cuVSLAM](https://developer.nvidia.com/isaac/cuvslam) offers GPU-accelerated Visual SLAM optimized for NVIDIA hardware. See [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html).
:::

### Navigation Success (Real Indoor)

| Navigation Stack | Success Rate | Collision Rate | Avg. Time |
|-----------------|--------------|----------------|-----------|
| [Nav2](https://nav2.org/) (default) | 92% | 3% | 45s |
| [Nav2](https://nav2.org/) (tuned) | 96% | 1% | 38s |
| [Nav2](https://nav2.org/) + [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | 94% | 2% | 52s |
| Learning-based | 89% | 5% | 35s |

---

## Vision-Language-Action (VLA) Model Benchmarks

:::info 🆕 New Section - December 2025
VLA models represent the cutting edge of robot learning, directly outputting actions from vision and language inputs.
:::

### Manipulation Task Success Rate (SIMPLER Benchmark)

| Model | Architecture | Task Success | Inference (Jetson Thor) | Open Source |
|-------|-------------|--------------|-------------------------|-------------|
| [GR00T N1.6](https://github.com/NVIDIA/Isaac-GR00T) ⭐ | VLA + Cosmos Reason | 89% | 25 Hz | ✅ Yes |
| [π₀ (pi-zero)](https://www.physicalintelligence.company/) | Diffusion VLA | 91% | 15 Hz | ❌ No |
| [OpenVLA OFT](https://openvla.github.io/) | VLA + LoRA | 82% | 50 Hz | ✅ Yes |
| [OpenVLA FAST](https://openvla.github.io/) | VLA + Tokenizer | 78% | 75 Hz | ✅ Yes |
| [RT-2-X](https://robotics-transformer2.github.io/) | VLA (55B params) | 85% | 3 Hz | ❌ No |
| [Helix](https://figure.ai/) (Figure AI) | Proprietary VLA | 94%* | 30 Hz | ❌ No |
| [Octo](https://octo-models.github.io/) | Diffusion Policy | 76% | 20 Hz | ✅ Yes |

⭐ = **Recommended for Open Development** | *Helix benchmark on Figure 03 hardware only

:::tip VLA Selection Guide (December 2025)
- **Open Development**: [GR00T N1.6](https://github.com/NVIDIA/Isaac-GR00T) with [Cosmos Reason](https://developer.nvidia.com/cosmos) for reasoning
- **Speed-Critical**: [OpenVLA FAST](https://openvla.github.io/) with 15× faster inference via action tokenization
- **Quality-First**: [OpenVLA OFT](https://openvla.github.io/) (25-50× faster than vanilla, better success rates)
- **Edge Deployment**: All models run on [Jetson Thor](https://developer.nvidia.com/blog/introducing-nvidia-jetson-thor-the-ultimate-platform-for-physical-ai/) (128GB, 2070 TFLOPS)
:::

---

## LLM Planning Benchmarks (Updated December 2025)

### SayCan-style Task Completion

| LLM Backbone | Task Success | Affordance Accuracy | Avg. Steps |
|-------------|--------------|---------------------|------------|
| [GPT-4o](https://openai.com/gpt-4) | 92% | 95% | 4.2 |
| [Claude 3.5 Sonnet](https://anthropic.com/claude) | 90% | 93% | 4.3 |
| [Claude 3 Opus](https://anthropic.com/claude) | 87% | 91% | 4.5 |
| [Gemini 2.0 Flash](https://deepmind.google/technologies/gemini/) | 88% | 90% | 4.4 |
| [Llama 3.1 405B](https://llama.meta.com/) | 84% | 88% | 4.8 |
| [Llama 3.2 90B](https://llama.meta.com/) | 80% | 85% | 5.0 |
| Llama 3.2 11B (local) | 68% | 72% | 5.8 |

### Code Generation for Robotics

| Model | Pass@1 (ROS 2 tasks) | Execution Success | Latency (API) |
|-------|----------------------|-------------------|---------------|
| [Claude 3.5 Sonnet](https://anthropic.com/claude) | 82% | 75% | 1.5s |
| [GPT-4o](https://openai.com/gpt-4) | 80% | 73% | 1.8s |
| [Gemini 2.0 Pro](https://deepmind.google/technologies/gemini/) | 78% | 70% | 2.0s |
| [Codestral](https://mistral.ai/news/codestral/) | 72% | 62% | 1.2s |
| [Code Llama 70B](https://llama.meta.com/) | 65% | 52% | 0.6s (local) |

---

## Simulation Benchmarks

### Physics Simulation Performance

| Simulator | Robot Load Time | Physics FPS | Render FPS | GPU Memory |
|-----------|----------------|-------------|------------|------------|
| Gazebo Classic | 8s | 1,000 | 60 | 1.5GB |
| Gazebo Harmonic | 5s | 1,200 | 60 | 2.0GB |
| Isaac Sim | 45s | 10,000+ | 60 | 8GB |
| MuJoCo | 0.5s | 50,000+ | N/A | 0.1GB |
| PyBullet | 1s | 5,000 | 60 | 0.5GB |

### Parallel Environment Scaling (Isaac Gym)

| Environments | FPS (RTX 3090) | FPS (RTX 4090) | Speedup |
|--------------|----------------|----------------|---------|
| 64 | 15,000 | 22,000 | 1.47x |
| 256 | 45,000 | 75,000 | 1.67x |
| 1024 | 120,000 | 210,000 | 1.75x |
| 4096 | 280,000 | 520,000 | 1.86x |

---

## Edge Deployment Benchmarks

### Jetson Orin Performance

| Task | Model | FPS (FP32) | FPS (FP16) | FPS (INT8) | Power |
|------|-------|------------|------------|------------|-------|
| Detection | YOLOv8s | 65 | 120 | 180 | 25W |
| Detection | YOLOv8m | 35 | 65 | 95 | 30W |
| Segmentation | YOLOv8s-seg | 48 | 95 | 140 | 28W |
| Depth | MiDaS Small | 25 | 45 | 65 | 22W |
| ASR | Whisper Small | 0.4x RT | 0.8x RT | 1.2x RT | 35W |
| LLM | Llama 3 8B (4-bit) | 12 tok/s | - | - | 45W |

### Latency Breakdown (Full Pipeline)

| Stage | Time (ms) | Hardware |
|-------|-----------|----------|
| Image Capture | 5 | Camera USB |
| Detection | 15 | Jetson TensorRT |
| Depth Processing | 8 | Jetson CUDA |
| Point Cloud | 12 | Jetson CPU |
| Planning | 25 | Workstation |
| Network Transfer | 5 | WiFi 6 |
| Control | 2 | Jetson RT |
| **Total** | **72ms** | - |

---

## How to Use These Benchmarks

### Selecting Algorithms

1. **Identify constraints**: Latency budget, hardware, accuracy requirements
2. **Find relevant table**: Match task to benchmark category
3. **Filter by hardware**: Use Jetson columns for edge, RTX for workstation
4. **Compare tradeoffs**: Speed vs accuracy, model size vs performance
5. **Validate locally**: Run your own benchmarks before deployment

### Reproducing Results

All benchmarks use standard evaluation protocols:
- **Detection**: COCO evaluation script, IoU=0.5:0.95
- **Depth**: NYU Depth v2 standard splits
- **ASR**: LibriSpeech test-clean/test-other
- **SLAM**: TUM RGB-D benchmark tools
- **Navigation**: 100 randomized start/goal pairs

### Reporting Your Own Results

When benchmarking your implementations:
- Report mean ± std over 3+ runs
- Specify exact hardware (GPU, driver version)
- Include batch size and precision (FP32/FP16/INT8)
- Link to reproducible code/config

---

:::info Connection to Textbook
These benchmarks inform algorithm selection throughout the textbook. Refer to specific sections for implementation details:
- Detection: M3-C2-S3
- Depth: M3-C2-S6
- Speech: M4-C1-S2
- Planning: M3-C2-S1
- SLAM: M2-C2-S7
:::

