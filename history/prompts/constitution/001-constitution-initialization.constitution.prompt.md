---
id: 001
title: Constitution Initialization
stage: constitution
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: Muhammad-Junaid-Sajjad
command: /sp.constitution
labels: ["constitution", "hackathon", "initialization", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
/sp.constitution
# Constitution: "Physical AI & Humanoid Robotics" Textbook (Hackathon I)

## Article I — Mission (Non-Negotiable)
Create a Docusaurus textbook for a capstone quarter on **Physical AI & Humanoid Robotics** with the central goal: **Bridge the gap between the digital brain and the physical body** by teaching students to design, simulate, and deploy humanoid-robot behaviors in simulated and real-world environments using the provided course stack.

## Article II — Scope Lock (Use Only the Provided Brief)
All content MUST be derived strictly from the Hackathon-provided brief. Do NOT introduce new tools, platforms, libraries, robots, hardware, or costs beyond what is explicitly present in the brief.

## Article III — Required Structure (4×3×7) + Completion Rule
The textbook MUST follow this exact hierarchy:
- 4 Modules | 3 Chapters per Module | 7 Sections per Chapter.
- Total: **84 Sections**.
- Completion rule: the project is only "done" when all 84 sections exist and the site builds/deploys to GitHub Pages.

## Article IV — Module Truth (Must Match Brief)
1. Module 1: The Robotic Nervous System (ROS 2)
2. Module 2: The Digital Twin (Gazebo & Unity)
3. Module 3: The AI-Robot Brain (NVIDIA Isaac™)
4. Module 4: Vision-Language-Action (VLA)

## Article V — Narrative Thread
Every section MUST connect back to: **Embodied Intelligence**, the **Digital Twin workflow**, and **Sim-to-Real transfer**. This is the "spine" of the book.

## Article VI — Hardware Truth (The Technical Law)
- **Workstation:** NVIDIA RTX 4070 Ti (12GB) or higher (Ideal: 3090/4090 24GB), Intel i7 (13th Gen+), 64GB DDR5, Ubuntu 22.04 LTS.
- **Edge Kit:** NVIDIA Jetson Orin Nano (8GB) or NX (16GB), Intel RealSense D435i/D455, USB IMU, ReSpeaker Mic Array.
- **Robot Lab:** Unitree G1 (Humanoid), Unitree Go2 (Proxy), or Robotis OP3.
- **Cloud (Ether Lab):** AWS g5.2xlarge ($205/quarter budget).

## Article VII — Latency Trap Rule
Documentation MUST emphasize: Simulation/Training happens on the Workstation/Cloud; model weights are then **flashed** to the local Edge Kit for inference. Warn against direct cloud-to-robot control.

## Article VIII — Capstone Definition
All content builds toward:
A **simulated humanoid** receiving a voice command, performing cognitive planning, executing ROS 2 actions, navigating obstacles, and manipulating objects.

- The capstone MUST be fully achievable in **simulation first**.
- Physical robot deployment is an **extension**, not a requirement, and must follow Sim-to-Real principles.

## Article IX — Delivery Toolchain
Written in Docusaurus, deployed to GitHub Pages, using **Spec-Kit Plus** and **Claude Code** for execution.

## Article X — Information Density + Quality Bar
- Crystal-clear, technical, and educational.
- **No fluff:** Every section must advance the provided learning outcomes and capstone readiness.
- **Technical Depth:** Include code/config where appropriate; conceptual sections must still be technical and capstone-relevant.
- Maintain consistency in terminology exactly as used in the brief (e.g., ROS 2, VSLAM, URDF, USD, Whisper, VLA).

## Article XI — Repository & File System Rules
- Enforce directory schema: `docs/M[1-4]/C[1-3]/S[1-7].md`.
- Sidebars/navigation MUST reflect the 4×3×7 hierarchy exactly.

## Article XII — Consistency & Assessment Alignment Check
The final section of every module MUST include a check verifying:
- Sim-to-Real narrative adherence.
- Capstone pipeline support (Voice → Plan → Actions → Navigate → Vision → Manipulation).
- Hardware constraint respect.
- Explicit alignment between **course assessments** and the section IDs that prepare students for them.

## Article XIII — Hackathon Bonus Scope (Non-Core, Optional)
The following features are OPTIONAL enhancements tied to hackathon bonus points and MUST remain strictly non-intrusive to the core textbook:

- Integrated RAG chatbot for answering book content.
- Optional user authentication (Better-Auth).
- Content personalization based on user background.
- Optional Urdu translation toggle.

These features MUST NOT alter the 4×3×7 structure, core content scope, or completion rule.
```

## Response snapshot

Constitution v1.0.0 successfully created at `.specify/memory/constitution.md` with 13 core principles derived from the hackathon brief:

**Key Principles Established:**
1. Mission Statement: Bridge digital brain to physical body through Docusaurus textbook
2. Scope Lock: Only use tools/hardware from hackathon brief (ROS 2, Gazebo, Unity, Isaac, etc.)
3. Required Structure: 4×3×7 hierarchy (84 total sections) - completion gate
4. Module Truth: ROS 2 → Digital Twin → Isaac AI → VLA (pedagogical sequence)
5. Narrative Thread: Every section ties to Embodied Intelligence, Digital Twin, Sim-to-Real
6. Hardware Truth: Workstation specs (RTX 4070 Ti+), Edge Kit (Jetson Orin), Robot Lab (Unitree G1/Go2)
7. Latency Trap Rule: Train on workstation/cloud, flash weights to edge (no cloud control loops)
8. Capstone Definition: Voice → Plan → Actions → Navigate → Manipulate (simulation-first, hardware-optional)
9. Delivery Toolchain: Docusaurus + GitHub Pages + Spec-Kit Plus
10. Information Density: Technical depth, no fluff, runnable code in every section
11. Repository Rules: `docs/M[1-4]/C[1-3]/S[1-7].md` schema enforcement
12. Assessment Alignment: Final module section maps assessments to section IDs
13. Hackathon Bonus Scope: RAG chatbot, auth, personalization, Urdu—strictly optional plugins

**Sync Impact Report embedded in file:**
- Version: 1.0.0 (Initial ratification)
- Templates validated: plan-template.md, spec-template.md, tasks-template.md all align
- No follow-up TODOs
- All placeholders resolved

**Governance Structure:**
- Amendment procedure requires ADR documentation
- Semantic versioning for constitution changes
- Compliance review mandatory for all PRs

## Outcome

- ✅ Impact: Constitution v1.0.0 ratified; project governance established with 13 non-negotiable principles tailored to Physical AI & Humanoid Robotics textbook development
- 🧪 Tests: Template alignment validated (plan, spec, tasks templates checked against new principles)
- 📁 Files: `.specify/memory/constitution.md` created (253 lines); Sync Impact Report embedded
- 🔁 Next prompts: `/sp.specify <feature-name>` to begin feature specification, `/sp.plan` for architecture design, or direct textbook content development
- 🧠 Reflection: Constitution successfully bridges hackathon requirements with Spec-Kit Plus methodology; 4×3×7 structure provides clear completion criteria; hardware/software constraints explicitly documented to prevent scope creep

## Evaluation notes (flywheel)

- Failure modes observed: None. All template placeholders successfully filled from user input. Directory structure validation passed.
- Graders run and results (PASS/FAIL): PASS - Constitution Check section in plan-template.md aligns with new principles; PASS - Spec template requirements structure matches scope lock enforcement; PASS - Tasks template organization supports 4×3×7 hierarchy
- Prompt variant (if applicable): Standard constitution initialization from detailed user requirements (hackathon brief as source of truth)
- Next experiment (smallest change to try): Begin feature specification for first textbook module (M1: ROS 2 Nervous System) or initialize Docusaurus project structure per Article IX delivery toolchain requirements
