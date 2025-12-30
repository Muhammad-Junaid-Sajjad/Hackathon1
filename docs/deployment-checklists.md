---
id: deployment-checklists
title: Deployment Checklists
sidebar_position: 103
keywords: ['deployment', 'checklist', 'production', 'safety', 'verification']
---

# ✅ Deployment Checklists

These checklists ensure safe, reliable deployment of Physical AI and humanoid robotics systems. Use them before moving from simulation to real hardware, and before any production deployment.

---

## Pre-Deployment Phase

### 1. Development Environment Checklist

Before beginning hardware integration:

- [ ] **ROS 2 Humble** installed and tested (`ros2 doctor`)
- [ ] **Gazebo Ionic** launches without errors
- [ ] **Isaac Sim** runs smoothly (if using NVIDIA sim)
- [ ] All **Python dependencies** installed in virtual environment
- [ ] **CUDA** and **cuDNN** versions match PyTorch requirements
- [ ] **TensorRT** installed for inference optimization
- [ ] Git repository initialized with proper `.gitignore`
- [ ] CI/CD pipeline configured for automated testing

### 2. Simulation Validation Checklist

Before transitioning to real hardware:

- [ ] All algorithms tested in **physics simulation** for 1000+ episodes
- [ ] **Domain randomization** applied (lighting, textures, physics)
- [ ] **Sensor noise models** configured to match real hardware
- [ ] **Control latency simulation** enabled (add 10-50ms delays)
- [ ] Success rate in simulation exceeds **95%** on test scenarios
- [ ] **Edge cases** tested (collisions, failures, timeouts)
- [ ] Simulation runs **deterministically** with fixed seeds
- [ ] Performance profiled and bottlenecks addressed

### 3. Model Validation Checklist

Before deploying ML models:

- [ ] Model exported to **ONNX** format successfully
- [ ] **TensorRT** engine generated and tested
- [ ] Inference latency measured: ___ms (target: under 50ms)
- [ ] Model accuracy validated on held-out test set: ___%
- [ ] **Quantization** tested (FP16/INT8) if latency-critical
- [ ] Input preprocessing matches training pipeline exactly
- [ ] Output postprocessing verified (NMS, thresholds)
- [ ] Model versioned and checksum recorded

---

## Hardware Integration Phase

### 4. Hardware Setup Checklist

Physical hardware preparation:

- [ ] All hardware components received and inspected
- [ ] **Power supplies** rated for peak load (add 20% margin)
- [ ] **Emergency stop** (E-Stop) installed and tested
- [ ] **Cable management** organized, strain relief applied
- [ ] **Mounting** secure, vibration-resistant
- [ ] **Cooling** adequate for sustained operation
- [ ] All firmware **updated** to latest stable versions
- [ ] Serial numbers and configurations documented

### 5. Sensor Calibration Checklist

Ensure accurate perception:

- [ ] **Camera intrinsics** calibrated (reprojection error under 0.5px)
- [ ] **Camera extrinsics** calibrated to robot base frame
- [ ] **IMU** calibrated using Allan Variance (bias, noise density)
- [ ] **LiDAR-camera** extrinsic calibration verified
- [ ] **Force/torque sensor** zeroed and calibrated
- [ ] Calibration files version-controlled and timestamped
- [ ] Recalibration schedule established (monthly/quarterly)
- [ ] Validation dataset collected for drift detection

### 6. Communication Verification Checklist

Ensure reliable data flow:

- [ ] All ROS 2 topics publishing at expected rates
- [ ] **QoS settings** configured (reliability, durability, deadline)
- [ ] Network latency measured: ___ms (target: under 5ms)
- [ ] **DDS domain ID** set to avoid conflicts
- [ ] TF tree complete and transforms publishing
- [ ] No message drops under load (tested with `ros2 topic hz`)
- [ ] Bandwidth utilization within limits
- [ ] Failover mechanisms tested (topic timeout handling)

---

## Safety Verification Phase

### 7. Safety Systems Checklist

**CRITICAL: Complete before any autonomous operation**

- [ ] **E-Stop** immediately halts all motion (tested 3+ times)
- [ ] **Software watchdog** detects and handles node failures
- [ ] **Joint limits** enforced in software AND hardware
- [ ] **Velocity limits** configured for safe operation
- [ ] **Force limits** prevent excessive contact forces
- [ ] **Collision detection** stops motion on unexpected contact
- [ ] **Workspace boundaries** defined and enforced
- [ ] **Human detection** zones configured (if collaborative)
- [ ] Safety-rated PLC/controller used (if required by standards)
- [ ] Safety documentation prepared for audit

### 8. Failure Mode Analysis Checklist

Identify and mitigate risks:

| Failure Mode | Probability | Severity | Mitigation | Status |
|-------------|-------------|----------|------------|--------|
| Communication loss | Medium | High | Watchdog timeout → safe stop | [ ] |
| Sensor failure | Low | High | Redundancy/graceful degradation | [ ] |
| Power loss | Low | Critical | UPS backup, controlled shutdown | [ ] |
| Collision | Medium | High | Force limits, E-Stop | [ ] |
| Software crash | Medium | Medium | Supervisor restart, state recovery | [ ] |
| Thermal overload | Low | Medium | Temperature monitoring, throttling | [ ] |

- [ ] All high-severity failure modes have mitigations
- [ ] Recovery procedures documented and tested
- [ ] Failure logging and alerting configured

### 9. Compliance Checklist

Regulatory and standards compliance:

- [ ] **ISO 10218** requirements reviewed (industrial robots)
- [ ] **ISO 13482** requirements reviewed (personal care robots)
- [ ] **ISO/TS 15066** requirements reviewed (collaborative robots)
- [ ] Risk assessment completed per **ISO 12100**
- [ ] CE marking requirements understood (if applicable)
- [ ] Documentation package prepared for certification
- [ ] Training requirements identified for operators
- [ ] Incident reporting procedures established

---

## Pre-Launch Verification Phase

### 10. System Integration Checklist

Verify complete system operation:

- [ ] All nodes launch successfully via launch file
- [ ] **Perception pipeline** produces valid outputs
- [ ] **Planning pipeline** generates executable trajectories
- [ ] **Control pipeline** tracks trajectories accurately
- [ ] **State machine** transitions correctly
- [ ] **Voice interface** recognizes commands (if applicable)
- [ ] **Error handling** triggers appropriate recovery
- [ ] End-to-end latency measured: ___ms (target: under 200ms)
- [ ] System runs for 1+ hour without crashes

### 11. Performance Validation Checklist

Quantitative verification:

| Metric | Target | Measured | Pass |
|--------|--------|----------|------|
| Object detection accuracy | >90% | ___% | [ ] |
| Grasp success rate | >85% | ___% | [ ] |
| Navigation success rate | >95% | ___% | [ ] |
| Command recognition accuracy | >90% | ___% | [ ] |
| End-to-end task success | >80% | ___% | [ ] |
| Mean task completion time | under 60s | ___s | [ ] |
| Control loop frequency | 100Hz+ | ___Hz | [ ] |
| Perception latency | under 50ms | ___ms | [ ] |

- [ ] All critical metrics meet targets
- [ ] Performance documented and baselined
- [ ] Regression test suite created

### 12. Operational Readiness Checklist

Ready for deployment:

- [ ] **Runbook** created with common procedures
- [ ] **Troubleshooting guide** documents known issues
- [ ] **Monitoring dashboard** configured (Grafana, etc.)
- [ ] **Alerting** set up for critical failures
- [ ] **Log aggregation** configured for debugging
- [ ] **Backup procedures** documented and tested
- [ ] **Update procedures** documented (OTA if applicable)
- [ ] On-call rotation established (if 24/7 operation)

---

## Deployment Day Checklist

### 13. Final Pre-Deployment Verification

On the day of deployment:

- [ ] Environment inspected for hazards
- [ ] All personnel briefed on safety procedures
- [ ] E-Stop locations communicated to all present
- [ ] First aid kit accessible
- [ ] Emergency contacts posted
- [ ] Video recording equipment ready (for debugging)
- [ ] Rollback plan prepared if issues arise
- [ ] Success criteria defined for go/no-go decision

### 14. Staged Deployment Checklist

Incremental activation:

**Stage 1: Static Verification**
- [ ] Power on all systems
- [ ] Verify all nodes running (`ros2 node list`)
- [ ] Check sensor data quality
- [ ] Verify TF tree (`ros2 run tf2_tools view_frames`)
- [ ] E-Stop test (with robot powered)

**Stage 2: Limited Motion**
- [ ] Enable robot in **teach mode** (reduced speed)
- [ ] Manually jog all joints through range
- [ ] Verify joint position feedback accuracy
- [ ] Test gripper open/close
- [ ] Verify force/torque sensing

**Stage 3: Scripted Operation**
- [ ] Run predefined test trajectories
- [ ] Verify perception detects test objects
- [ ] Execute simple pick-and-place
- [ ] Monitor all metrics during operation
- [ ] No anomalies observed

**Stage 4: Autonomous Operation**
- [ ] Enable full autonomous mode
- [ ] Execute complete task scenarios
- [ ] Monitor for 30+ minutes
- [ ] Document any issues
- [ ] Declare deployment successful

---

## Post-Deployment Phase

### 15. Monitoring Checklist

Ongoing operational monitoring:

- [ ] **CPU/GPU utilization** within normal range
- [ ] **Memory usage** stable (no leaks)
- [ ] **Disk space** sufficient for logs
- [ ] **Network traffic** normal
- [ ] **Error rates** below threshold
- [ ] **Task success rate** maintained
- [ ] **Latency percentiles** (p50, p95, p99) tracked
- [ ] Weekly review of metrics and logs

### 16. Maintenance Checklist

Regular maintenance tasks:

**Daily:**
- [ ] Visual inspection of robot and workspace
- [ ] Check E-Stop functionality
- [ ] Review error logs from previous 24 hours
- [ ] Verify backup systems operational

**Weekly:**
- [ ] Clean sensors (cameras, LiDAR)
- [ ] Check cable connections
- [ ] Review performance metrics
- [ ] Update documentation with learnings

**Monthly:**
- [ ] Recalibrate sensors if drift detected
- [ ] Apply software updates (test first)
- [ ] Review and update failure mode analysis
- [ ] Conduct safety drill

**Quarterly:**
- [ ] Full system calibration
- [ ] Hardware inspection and preventive maintenance
- [ ] Performance benchmark vs. baseline
- [ ] Safety audit and documentation review

---

## Quick Reference Cards

### Emergency Procedures

```
┌─────────────────────────────────────────────────────┐
│              EMERGENCY PROCEDURES                   │
├─────────────────────────────────────────────────────┤
│                                                     │
│  1. PRESS E-STOP (Big Red Button)                  │
│                                                     │
│  2. CLEAR THE AREA of personnel                    │
│                                                     │
│  3. ASSESS the situation                           │
│     - Any injuries? → Call emergency services      │
│     - Property damage? → Document and report       │
│                                                     │
│  4. DO NOT RESET until cause identified            │
│                                                     │
│  5. REPORT incident to safety officer              │
│                                                     │
│  Emergency Contact: _______________                │
│  Safety Officer: _________________                 │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### Startup Sequence

```
┌─────────────────────────────────────────────────────┐
│              STARTUP SEQUENCE                       │
├─────────────────────────────────────────────────────┤
│                                                     │
│  1. INSPECT workspace for hazards                  │
│  2. POWER ON control systems                       │
│  3. VERIFY E-Stop not engaged                      │
│  4. LAUNCH ROS 2 stack:                            │
│     ros2 launch humanoid_bringup full_system.launch│
│  5. WAIT for all nodes (check ros2 node list)      │
│  6. VERIFY sensors (check RViz)                    │
│  7. ENABLE robot (release brakes)                  │
│  8. HOME robot to known configuration              │
│  9. RUN self-test routine                          │
│  10. CONFIRM ready status                          │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### Shutdown Sequence

```
┌─────────────────────────────────────────────────────┐
│              SHUTDOWN SEQUENCE                      │
├─────────────────────────────────────────────────────┤
│                                                     │
│  1. STOP all autonomous operations                 │
│  2. MOVE robot to home/park position               │
│  3. ENGAGE brakes (if applicable)                  │
│  4. STOP ROS 2 nodes (Ctrl+C or systemctl)         │
│  5. SAVE logs if needed                            │
│  6. POWER OFF robot systems                        │
│  7. POWER OFF control computers (optional)         │
│  8. ENGAGE E-Stop for overnight                    │
│  9. SECURE workspace                               │
│                                                     │
└─────────────────────────────────────────────────────┘
```

---

## Checklist Templates

### Download Links

Use these templates for your deployment:

- [ ] Pre-Deployment Checklist (PDF)
- [ ] Safety Verification Checklist (PDF)
- [ ] Deployment Day Checklist (PDF)
- [ ] Maintenance Schedule Template (Excel)
- [ ] Incident Report Form (Word)

---

:::warning Important
These checklists are guidelines. Adapt them to your specific application, environment, and regulatory requirements. Always consult with safety professionals for high-risk deployments.
:::

:::info Connection to Capstone
The capstone humanoid assistant project should complete ALL checklists before real-world deployment. Simulation-only demonstrations can skip hardware-specific items but should still address safety and system integration checks.
:::

