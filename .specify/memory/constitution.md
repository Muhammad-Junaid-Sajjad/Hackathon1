<!--
SYNC IMPACT REPORT
==================
Version Change: 1.3.0 → 2.0.0 (The 2025 Frontier Standard)
Rationale: Synchronizing official project "Law" with the implemented 2025 technical stack (ROS 2 Kilted Kaiju, Ubuntu 24.04, RTX 50-series).

Modified Principles:
- Article II & IV: Middleware and AI Platform versions updated to 2025 standards.
- Article VI: Hardware baseline updated to RTX 5080/6080 and NVIDIA Thor platform.
- Article III & XI: Formally acknowledging the expansion of the 4x3x7 structure to include 2025 frontier research sections (87 total sections).

New Standards:
- OS: Ubuntu 24.04 LTS (Noble Numbat)
- Middleware: ROS 2 Kilted Kaiju (2025)
- Sim: Gazebo Ionic (9.x)
- GPU Target: RTX 50xx series (Blackwell)
- Vision: YOLOv11 benchmarks

Follow-up TODOs:
- Document the 3 extra sections in spec.md
- Create ADR-001 for the 2025 migration
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Mission Statement (Non-Negotiable)
Create a Docusaurus textbook for a capstone quarter on **Physical AI & Humanoid Robotics** with the central goal: **Bridge the gap between the digital brain and the physical body** by teaching students to design, simulate, and deploy humanoid-robot behaviors in simulated and real-world environments using the provided course stack.

**Rationale**: This mission ensures all development efforts remain focused on the educational outcome—students must be able to understand embodied intelligence, work with digital twins, and execute sim-to-real workflows. Any feature, content, or tooling decision that does not advance this mission MUST be rejected.

### II. Scope Lock (Strict Boundary)
All content MUST be derived strictly from the Hackathon-provided brief. Do NOT introduce new tools, platforms, libraries, robots, hardware, or costs beyond what is explicitly present in the brief.

**Rationale**: Prevents scope creep and ensures the textbook remains implementable within the specified hardware, software, and budget constraints. Students and instructors must be able to replicate the environment without encountering undocumented dependencies.

**Enforcement**: Every section, code example, and tutorial MUST reference only the approved stack:
- **Middleware**: ROS 2 Kilted Kaiju (2025 Standard)
- **Simulation**: Gazebo Ionic (9.x), Unity with ROS Integration
- **AI Platform**: NVIDIA Isaac Sim (2025.x), Isaac Lab, NVIDIA Thor
- **Perception**: Intel RealSense SDK 2.0, OpenCV, NVIDIA VPI 3.x, YOLOv11
- **Language Models**: Whisper (ASR), RT-2/Octo/VILA (VLA policies)
- **Hardware**: See Article VI

### III. Required Structure (4×3×7+) + Completion Rule
The textbook MUST follow this exact hierarchy:
- **4 Modules** | **3 Chapters per Module** | **7 Sections per Chapter**
- **Exception**: 3 Frontier Research sections added (87 Total Sections) to cover 2025 VLA and Thor platform advancements.

**Completion Rule**: The project is only "done" when all 87 sections exist and the site builds/deploys to GitHub Pages without errors.

**Rationale**: This structure provides a predictable, scalable learning architecture that maps directly to a 10-week quarter (approximately 2 sections per week + review). The 4×3×7 hierarchy ensures consistent depth, prevents unbalanced modules, and makes progress measurable.

**Enforcement**:
- Directory schema: `docs/M[1-4]/C[1-3]/S[1-7].md`
- Sidebars/navigation MUST reflect the 4×3×7 hierarchy exactly
- No module may be marked "complete" unless all 21 sections exist
- GitHub Pages deployment MUST succeed before final sign-off

### IV. Module Truth (Immutable Curriculum Map)
1. **Module 1**: The Robotic Nervous System (ROS 2) - Middleware for robot control, nodes, topics, services, URDF
2. **Module 2**: The Digital Twin (Gazebo & Unity) - Physics simulation, environment building, sensor simulation
3. **Module 3**: The AI-Robot Brain (NVIDIA Isaac™) - Isaac Sim, Isaac ROS, VSLAM, Nav2, AI-powered perception
4. **Module 4**: Vision-Language-Action (VLA) - Whisper ASR, LLM planning, end-to-end humanoid control

**Detailed Module Descriptions**:

**Module 1: The Robotic Nervous System (ROS 2)**
- ROS 2 architecture and core concepts (Nodes, Topics, Services, Actions)
- Bridging Python Agents to ROS controllers using rclpy
- Understanding URDF (Unified Robot Description Format) for humanoids
- Build custom ROS 2 packages with Python
- Launch files and parameter management

**Module 2: The Digital Twin (Gazebo & Unity)**
- Gazebo simulation environment setup
- URDF and SDF robot description formats
- Physics simulation and sensor simulation (LiDAR, Depth Cameras, IMUs)
- High-fidelity rendering and human-robot interaction in Unity
- Sim-to-Real transfer techniques

**Module 3: The AI-Robot Brain (NVIDIA Isaac™)**
- NVIDIA Isaac Sim: Photorealistic simulation and synthetic data generation
- Isaac ROS: Hardware-accelerated VSLAM (Visual SLAM) and navigation
- Nav2: Path planning for bipedal humanoid movement
- Reinforcement learning for robot control
- AI-powered perception and manipulation

**Module 4: Vision-Language-Action (VLA)**
- Voice-to-Action: Using OpenAI Whisper for voice commands
- Cognitive Planning: Using LLMs to translate natural language into ROS 2 action sequences
- Multi-modal interaction: speech, gesture, vision
- The Autonomous Humanoid: Voice command → Path planning → Navigation → Object identification → Manipulation

**Rationale**: These four modules represent the technical scaffolding required to build embodied intelligence systems. The order is pedagogically deliberate:
1. First, students learn the communication backbone (ROS 2)
2. Then, they master simulation and virtual prototyping (Digital Twin)
3. Next, they integrate AI/ML for perception and control (Isaac™)
4. Finally, they combine vision, language, and action into intelligent behaviors (VLA)

**Enforcement**: No content may be added to a module that does not directly support its title and core focus. Cross-module dependencies MUST be explicitly documented.

### V. Narrative Thread (The Spine)
Every section MUST connect back to:
- **Embodied Intelligence**: How does this content enable intelligent behavior in a physical body?
- **Digital Twin Workflow**: How does this content support simulation-first development?
- **Sim-to-Real Transfer**: How does this content prepare students to deploy from simulation to hardware?

**Rationale**: This narrative thread prevents isolated, disconnected topics. Students should always understand "why" they are learning a concept and how it fits into the larger goal of building humanoid robots that think and act.

**Enforcement**: Each section MUST include a "Connection to Capstone" subsection that explicitly ties the material to the final project pipeline (Voice → Plan → Actions → Navigate → Vision → Manipulation).

### VI. Hardware Truth (The Technical Law)

**This course sits at the intersection of three heavy computational loads:**
1. Physics Simulation (Isaac Sim 2025/Gazebo Ionic)
2. Visual Perception (YOLOv11/VSLAM)
3. Generative AI (VILA/VLA Policies)

#### Workstation (Required - The "Digital Twin" Rig)

This is the most critical component. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities.

| Component | Minimum Specification | Ideal Specification | Purpose |
|-----------|----------------------|---------------------|---------|
| **GPU** | NVIDIA RTX 5080 (16GB VRAM) | RTX 5090/6080 (24GB-32GB VRAM) | Blackwell architecture, 2025 VLA models |
| **CPU** | Intel Core i9 (14th Gen+) or AMD Ryzen 9 | Same | Physics (Ionic/Isaac Sim) |
| **RAM** | 64GB DDR5 (minimum) | 128GB DDR5 | Multi-humanoid simulation, VLA context |
| **Storage** | 2TB NVMe SSD Gen 5 | Same | Fast I/O, massive USD datasets |
| **OS** | Ubuntu 24.04 LTS (Noble) | Same | Native ROS 2 Kilted Kaiju support |

**Note**: Dual-booting or dedicated Linux machines are mandatory for kernel-level performance in simulation-to-robot cycles.

#### Edge Kit (The "Physical AI" Brain)

For deploying code to physical hardware and understanding resource constraints.

| Component | Model | Price | Purpose |
|-----------|-------|-------|---------|
| **Compute** | NVIDIA Jetson Thor (GTC 2025) | ~$499 | Humanoid-specific SoC for VLA inference |
| **Camera** | Intel RealSense D435i/D455 | ~$349 | RGB + Depth + IMU for SLAM |
| **Audio** | ReSpeaker USB Mic Array v2.0 | ~$69 | Far-field voice commands (Whisper) |
| **Storage** | 128GB High-endurance microSD | ~$30 | OS and runtime storage |
| **Total** | | ~$950 per kit | |

**Alternative**: Jetson Orin NX (16GB) for more demanding workloads.

#### Robot Lab Options

**Option A: The "Proxy" Approach (Recommended for Budget)**
- Robot: Unitree Go2 Edu ($1,800 - $3,000)
- Pros: Highly durable, excellent ROS 2 support, affordable
- Cons: Not a biped (quadruped)

**Option B: The "Miniature Humanoid" Approach**
- Robot: Unitree G1 (~$16,000) or Robotis OP3 (~$12,000)
- Budget Alternative: Hiwonder TonyPi Pro (~$600) - requires separate Jetson kit

**Option C: The "Premium" Lab (Sim-to-Real)**
- Robot: Unitree G1 Humanoid
- Why: Commercially available humanoid with open SDK for ROS 2 controllers

#### Cloud (The "Ether" Lab) - Optional

For students without local GPU access or overflow capacity.

| Service | Instance Type | Cost | Purpose |
|---------|---------------|------|---------|
| AWS | g5.2xlarge (A10G GPU, 24GB VRAM) | ~$1.50/hour | Isaac Sim, training |
| AWS | g6e.xlarge | Varies | Heavy workloads |
| **Quarterly Budget** | 120 hours usage | ~$205/quarter | Training and simulation |

#### Architecture Summary

```
┌─────────────────────────────────────────────────────────────┐
│                    Physical AI Lab Architecture              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────────┐    ┌───────────────┐    ┌────────────┐  │
│  │ Sim Rig       │    │ Edge Brain    │    │ Actuator   │  │
│  │ RTX 4080/4090 │───▶│ Jetson Orin   │───▶│ Unitree    │  │
│  │ Ubuntu 22.04  │    │ Nano/NX       │    │ Go2/G1     │  │
│  │ Runs Isaac    │    │ Runs Inference│    │ Motor Ctrl │  │
│  │ Sim, Gazebo   │    │ ROS 2 nodes   │    │            │  │
│  └───────────────┘    └───────────────┘    └────────────┘  │
│                                                             │
│  Cloud (Optional): AWS g5.2xlarge ~$205/quarter             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

**Rationale**: Hardware constraints are non-negotiable because they directly affect what algorithms can run, what models can be trained, and what latencies are achievable. Documenting exact specs ensures students know the minimum viable environment.

**Enforcement**:
- All tutorials MUST include hardware requirements at the top
- Performance benchmarks MUST specify the hardware used
- Alternative configurations (e.g., CPU-only fallback) MUST be clearly marked as limited-capability paths
- All hardware prices are estimates and subject to market changes

### VII. Latency Trap Rule (Cloud-to-Robot Warning)
Documentation MUST emphasize: **Simulation and training happen on the Workstation or Cloud; model weights are then flashed to the local Edge Kit for inference.** Warn against direct cloud-to-robot control loops.

**Rationale**: Network latency (50-200ms) makes cloud-based real-time control impractical for reactive tasks (e.g., balance, collision avoidance). Students must understand the difference between:
- **Training workflows**: Cloud-based, batch-oriented, latency-tolerant
- **Inference workflows**: Edge-based, real-time, latency-critical

**Enforcement**: Any section involving model deployment MUST include a "Latency Considerations" subsection explaining why the model runs locally on the Edge Kit and how weights are transferred.

### VIII. Capstone Definition (The North Star)
All content builds toward:

**Capstone Project**: A simulated humanoid that:
1. Receives a voice command (e.g., "Bring me the red block")
2. Performs cognitive planning (e.g., task decomposition, path planning)
3. Executes ROS 2 actions (e.g., navigation, manipulation)
4. Navigates obstacles (e.g., VSLAM, occupancy grids)
5. Manipulates objects (e.g., grasp planning, force control)

**Simulation-First Rule**: The capstone MUST be fully achievable in simulation (Gazebo or Isaac Sim) without requiring physical hardware.

**Physical Extension (Optional)**: Physical robot deployment follows sim-to-real principles and is an extension, not a requirement.

**Rationale**: This ensures all students can complete the capstone regardless of hardware access. Simulation-first development is industry-standard practice and teaches students to validate behavior in a controlled environment before risking expensive hardware.

**Enforcement**:
- Every module MUST include at least one "Capstone Checkpoint" section showing incremental progress toward the final demo
- Final assessment rubric MUST prioritize simulation performance; physical deployment is bonus points

### IX. Delivery Toolchain (Implementation Standard)
- **Static Site Generator**: Docusaurus (latest stable)
- **Deployment**: GitHub Pages via MCP browser tools
- **Debugging**: MCP browser for live testing and validation
- **Development Methodology**: Spec-Kit Plus workflow
- **Agent**: Claude Code (Anthropic)

**Rationale**: Docusaurus provides a modern, searchable, mobile-friendly documentation platform with excellent support for code examples. GitHub Pages ensures free, reliable hosting with CI/CD integration.

**MDX Handling**: Docusaurus uses MDX which can interpret curly braces `{}` as JSX expressions. The following patterns MUST be escaped or avoided:
- Curly braces in prose text (e.g., `{i}`, `{i+1}`) → Use HTML entities (`{i}` → `&#95;i` or remove braces)
- Comparison operators in prose text (e.g., `<`, `>`) → Use HTML entities (`&lt;`, `&gt;`)
- Subscripts in math notation (e.g., `x_i`) → Use HTML entities or rewrite as `x i` or `x_i` inside code blocks

**Enforcement**:
- All Markdown MUST be valid Docusaurus-flavored Markdown
- Code blocks MUST use triple backticks with language specifier
- Curly braces in prose tables/text MUST use HTML entities or be removed
- Comparison operators in prose text MUST use HTML entities
- Build process MUST pass linting and deployment checks before merge
- MCP browser testing MUST verify the deployed site loads correctly

### X. Pedagogical Excellence Standard (The Heart of the Textbook)

Every section MUST follow the **DEFINE → EXPLAIN → ILLUSTRATE → PRACTICE** framework to ensure complete understanding.

#### 10.1 Content Balance Requirements

Every section MUST contain a balanced mix of:

| Component | Percentage | Description |
|-----------|------------|-------------|
| **Theory** | 30-40% | Concept definitions, explanations, "why" behind each topic |
| **Practice** | 30-40% | Code examples, hands-on exercises, implementation steps |
| **Visuals** | 20-30% | Diagrams, flowcharts, mind maps, architecture illustrations |

**Rationale**: Learning requires multiple modalities. Pure theory is dry; pure practice lacks understanding; pure visuals lack depth. Balance ensures engagement and retention.

#### 10.2 Concept Introduction Pattern

Every new concept MUST follow this structure:

```
┌─────────────────────────────────────────────────────────────┐
│                    CONCEPT INTRODUCTION                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. WHAT IS IT? (Definition)                                │
│     - One clear sentence definition                         │
│     - Etymology/origin if helpful                           │
│                                                             │
│  2. WHY DO WE NEED IT? (Motivation)                         │
│     - Real-world problem it solves                          │
│     - What happens without it                               │
│                                                             │
│  3. HOW DOES IT WORK? (Explanation)                         │
│     - Step-by-step breakdown                                │
│     - Working principles                                    │
│     - Underlying mechanisms                                 │
│                                                             │
│  4. VISUAL REPRESENTATION (Diagram)                         │
│     - Architecture diagram OR                               │
│     - Flowchart OR                                          │
│     - Mind map OR                                           │
│     - Comparison table                                      │
│                                                             │
│  5. REAL-WORLD EXAMPLE (Concrete Application)               │
│     - Industry use case                                     │
│     - Relatable analogy                                     │
│                                                             │
│  6. HANDS-ON EXERCISE (Practice)                            │
│     - Working code example                                  │
│     - Expected output                                       │
│     - What to observe                                       │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### 10.3 Progressive Learning Structure

**Chapter Flow**: Each chapter MUST build progressively:
1. **Chapter Start**: Begin from absolute basics (assume no prior knowledge of chapter topic)
2. **Section Flow**: Each section builds on previous sections within the chapter
3. **Module Flow**: Each module builds on concepts from previous modules
4. **Standalone**: Despite building progressively, each chapter should be understandable in isolation with brief prerequisite summaries

**Section Opening Pattern**:
```markdown
## Prerequisites
- [Brief list of what reader should know]
- [Link to relevant previous section if needed]

## Learning Objectives
By the end of this section, you will be able to:
- [Objective 1 - action verb]
- [Objective 2 - action verb]
- [Objective 3 - action verb]

## Key Concepts
| Term | Definition |
|------|------------|
| [Term 1] | [One-line definition] |
| [Term 2] | [One-line definition] |
```

#### 10.4 Visual Requirements

Every section MUST include at least ONE of:
- **Architecture Diagram**: System components and data flow
- **Flowchart**: Decision processes or algorithms
- **Mind Map**: Concept relationships
- **Comparison Table**: Feature/option comparisons
- **Sequence Diagram**: Time-ordered interactions
- **Working Principle Illustration**: How something functions internally

**ASCII Diagram Standard** (for text-based diagrams):
```
Use box-drawing characters: ┌ ┐ └ ┘ │ ─ ├ ┤ ┬ ┴ ┼
Use arrows: → ← ↑ ↓ ↔ ⟶ ⟵
Use flow: ───▶ ◀───
```

#### 10.5 Example Requirements

Every concept MUST have:
1. **Simple Example**: Minimal code showing the concept in isolation
2. **Real-World Example**: How it's used in actual robotics/AI applications
3. **Expected Output**: What the reader should see when running the code
4. **Common Mistakes**: What can go wrong and how to fix it

#### 10.6 Engagement Standards

**Simplicity**: Use the simplest possible explanation first, then add complexity
**Clarity**: No jargon without definition; no acronyms without expansion
**Interest**: Connect every topic to exciting real-world robotics applications
**Curiosity**: End sections with "What's Next" teasers

**Tone Guidelines**:
- Write as if explaining to a smart friend, not a committee
- Use "you" and "we" to create connection
- Include "Why this matters" callouts
- Add "Fun Fact" or "Industry Insight" boxes for engagement

#### 10.7 Quality Checklist (Every Section)

Before marking a section complete, verify:
- [ ] Every new term is defined before use
- [ ] At least one diagram/visual exists
- [ ] At least one working code example exists
- [ ] Example output is shown
- [ ] Connection to capstone is explicit
- [ ] Prerequisites are listed
- [ ] Learning objectives are stated
- [ ] "What's Next" preview exists

#### 10.8 Terminology Consistency

- Use terms exactly as defined in the brief (e.g., ROS 2, VSLAM, URDF, USD, Whisper, VLA)
- Define acronyms on first use: "ROS 2 (Robot Operating System 2)"
- Maintain a glossary of terms used throughout the textbook

**Rationale**: Students are preparing for industry roles where precision matters. Vague language, inconsistent terminology, or shallow explanations undermine learning and professional readiness.

**Enforcement**:
- Every section MUST pass the 10.7 Quality Checklist before completion
- Peer review MUST verify pedagogical balance (theory/practice/visuals)
- Sections failing the checklist MUST be revised before publication

### XI. Repository & File System Rules
**Enforce Directory Schema**:
```
docs/
├── M1/
│   ├── C1/
│   │   ├── S1.md
│   │   ├── S2.md
│   │   ...
│   │   └── S7.md
│   ├── C2/
│   └── C3/
├── M2/
├── M3/
└── M4/
```

**Sidebars/Navigation**: MUST reflect the 4×3×7 hierarchy exactly.

**Rationale**: Consistent file organization makes content discoverable, maintainable, and automatable. Deviations break automation scripts and confuse contributors.

**Enforcement**:
- CI checks MUST validate directory structure
- File naming MUST follow `S[1-7].md` convention
- Sidebar generation MUST be automated from directory scan

### XII. Consistency & Assessment Alignment Check
The final section of every module MUST include a check verifying:
- **Sim-to-Real Narrative Adherence**: Does every section reference the digital twin workflow?
- **Capstone Pipeline Support**: Does the module contribute to (Voice → Plan → Actions → Navigate → Vision → Manipulation)?
- **Hardware Constraint Respect**: Are all examples runnable on the specified hardware?
- **Assessment Alignment**: Which course assessments does this module prepare students for, and which section IDs provide the necessary knowledge?

**Rationale**: This ensures pedagogical coherence and helps instructors map learning outcomes to assessments.

**Enforcement**: Module 4, Chapter 3, Section 7 MUST include an "Assessment Alignment Matrix" table linking assessments to section IDs.

### XIII. Hackathon Bonus Scope (Non-Core, Optional)
The following features are OPTIONAL enhancements tied to hackathon bonus points and MUST remain strictly non-intrusive to the core textbook:

- **RAG Chatbot**: Integrated assistant for answering book content questions
- **Authentication**: Optional Better-Auth integration for user tracking
- **Personalization**: Content recommendations based on user background
- **Urdu Translation**: Optional toggle for Urdu language support

**Rationale**: These features enhance user experience but are not required for educational outcomes. They must not delay core textbook completion.

**Enforcement**:
- Bonus features MUST NOT alter the 4×3×7 structure, core content scope, or completion rule
- Bonus features MUST be implemented as optional plugins or overlays
- Core textbook MUST build and deploy successfully with all bonus features disabled

## Governance

### Amendment Procedure
1. Proposed amendments MUST be documented in an Architecture Decision Record (ADR)
2. ADRs MUST include: context, decision, consequences, alternatives considered
3. Amendments require explicit approval from project stakeholders
4. Breaking changes require migration plan and communication strategy

### Versioning Policy
- **MAJOR**: Backward-incompatible changes (e.g., removing a module, changing structure)
- **MINOR**: New principles, sections, or materially expanded guidance
- **PATCH**: Clarifications, wording, typos, non-semantic refinements

### Compliance Review
- All PRs MUST verify compliance with this constitution
- Constitution violations MUST be explicitly justified in PR description
- Complexity introduced in violation of principles MUST be documented in plan.md "Complexity Tracking" section

### Runtime Guidance
For day-to-day development guidance beyond this constitution, see `CLAUDE.md` in the repository root.

**AMENDMENT NOTE (2025-12-27)**:
- **Version 1.1.0**: MDX Handling Rules Added
  - Fixed `ReferenceError: i is not defined` by escaping `{i}` as `&#95;i`
  - Added MDX handling guidelines to Article IX

- **Version 1.2.0**: Comprehensive Course Details Added
  - Updated Article IV with detailed module descriptions
  - Updated Article VI with complete hardware specifications

- **Version 1.3.0**: Pedagogical Excellence Standard Added
  - Article X completely rewritten with DEFINE → EXPLAIN → ILLUSTRATE → PRACTICE framework
  - Added content balance requirements: 30-40% Theory, 30-40% Practice, 20-30% Visuals
  - Added 6-step concept introduction pattern
  - Added 8-point quality checklist for every section
  - Created section template at `.specify/templates/section-template.md`
  - All 84 sections MUST be revised to meet new pedagogical standards

**Version**: 1.3.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-27
