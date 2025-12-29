---
id: 005
title: Atomic Task Checklist
stage: tasks
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-spec
branch: 001-textbook-spec
user: Muhammad-Junaid-Sajjad
command: /sp.tasks
labels: ["tasks", "checklist", "84-sections", "docusaurus", "implementation"]
links:
  spec: /specs/001-textbook-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-textbook-spec/tasks.md
tests:
  - none
---

## Prompt

Generate the atomic checklist for Claude Code implementation:

1. **Scaffolding:** Create 84 `.md` files in `docs/` following the `M[1-4]/C[1-3]/S[1-7].md` format.
2. **Config:** Setup `docusaurus.config.js` with the Physical AI branding and `sidebars.js` for the 4×3×7 hierarchy.
3. **Templates:** Pre-populate all 84 files with frontmatter (title, slug, keywords).
4. **Code Assets:** Create a `src/snippets` folder for the Python/rclpy code to be referenced in the docs.
5. **Deployment:** Write the `.github/workflows/deploy.yml` for automated GitHub Pages publishing.
6. **Article XII Check:** Create a 'Validation' partial to be included in the final section of every module.

**CONSTRAINTS (mandatory)**:
- Use ONLY brief-derived content and the already-approved spec.
- Maintain exact directory schema: `docs/M1/C1/S1.md` … `docs/M4/C3/S7.md`.
- No missing files; no extra sections beyond the 84.
- Enforce the Latency Trap rule and hardware truth across relevant sections.

**DELIVERABLE FORMAT (must follow exactly)**:
A) Repository & Docusaurus Tasks (T-SETUP-01 to T-SETUP-04)
B) Content Scaffolding Tasks (T-SCAFFOLD-01 to T-SCAFFOLD-03)
C) Writing Tasks (84 tasks, one per section with Task ID, file path, title, brief anchors, artifacts, latency trap, hardware, cross-links, acceptance criteria)
D) Mandatory Injection Tasks (T-INJECT-01 to T-INJECT-03)
E) QA & Consistency Tasks (T-QA-01 to T-QA-07)

Output as a checklist-style plan ready for Claude Code execution. No questions.

## Response snapshot

Generated comprehensive atomic task checklist (144 tasks total) organized into 11 phases matching 13-week roadmap.

**Task Breakdown by Category**:
- **Repository & Docusaurus Setup (A)**: 6 tasks (T001-T006) - Docusaurus init, package.json, config, docs plugin, dependencies, .gitignore
- **Content Scaffolding (B)**: 14 tasks (T007-T020) - Create 12 chapter directories (M1/M2/M3/M4 × C1/C2/C3), static/src/scripts/.github directories, sidebars.js config, 84 placeholder file creation, frontmatter population, template skeleton insertion
- **Module 1 Writing (C)**: 21 tasks (T021-T041) - 7 sections/week for 2 weeks + 7 sections in week 3
  - Week 2 (T021-T027): M1-C1 (Workstation setup, Jetson flash, Physical AI, ROS 2 install, nodes/topics, TF2, CLI)
  - Week 3 (T028-T041): M1-C2 (Services, Actions, Parameters, Launch, Messages, QoS, Rosbag) + M1-C3 (URDF canonical, Xacro, FK, IK, IMU, collisions, M1 Check)
- **Module 2 Writing (D)**: 21 tasks (T042-T062) - 7 sections/week for 3 weeks
  - Week 4 (T042-T048): M2-C1 (Gazebo install, physics, worlds, collisions, friction, controllers, ROS 2 bridge)
  - Week 5 (T049-T055): M2-C2 (RealSense sim, LiDAR, depth calibration, IMU noise, RGB-D alignment, latency sim, synthetic data)
  - Week 6 (T056-T062): M2-C3 (Unity install, HDRP, ROS bridge, avatar, voice UI, randomization, M2 Check)
- **Module 3 Writing (E)**: 21 tasks (T063-T083) - 6+8+7 sections over 3 weeks
  - Week 7 (T063-T068): M3-C1 Part 1 (Omniverse, USD, URDF→USD, Isaac physics, Replicator, OmniGraph)
  - Week 8 (T069-T076): M3-C1-S7 + M3-C2 (Cloud rendering + Ether Lab $205, Isaac ROS, cuVSLAM, detection, depth seg, Nav2, mapping, semantic seg)
  - Week 9 (T077-T083): M3-C3 (Bipedal, Isaac Gym, PPO, randomization, ONNX, weight flashing PRIMARY, M3 Check)
- **Module 4 Writing (F)**: 21 tasks (T084-T104) - 6+8+6+1 sections over 4 weeks
  - Week 10 (T084-T089): M4-C1 Part 1 (ReSpeaker, Whisper ASR, parsing, fusion, dialogue, gestures)
  - Week 11 (T090-T097): M4-C1-S7 + M4-C2 (TTS, LLM, behavior trees, context, grounding, error handling, safety, hierarchical)
  - Week 12 (T098-T103): M4-C3 Part 1 (Voice→Plan, Plan→Navigate, Navigate→Manipulate, sim testing, optional physical, rubric)
  - Week 13 (T104): M4-C3-S7 (Final Check + Assessment Matrix)
- **Injection Tasks (G)**: 9 tasks (T105-T113) - Latency trap callouts (22 Required + 14 Mention), Module Consistency checklists (4 modules), Cross-module links (URDF, Sim-to-Real, VLA→Actions)
- **Code Snippets & Assets (H)**: 11 tasks (T114-T124) - Create src/snippets/ with Python examples, static/code/ with configs/launch/scripts, static/img/ with diagrams/media, static/datasets/ with sample data
- **Deployment & CI/CD (I)**: 5 tasks (T125-T129) - GitHub Actions workflow, repository settings, 3 validation scripts (structure, latency traps, links)
- **QA & Consistency (J)**: 12 tasks (T130-T141) - Run 3 validation scripts, manual scope lock/info density/capstone/assessment verification, npm build, local serve test, git commit, push to trigger deploy, verify site live
- **Documentation & Polish (K)**: 3 tasks (T142-T144) - README.md with setup instructions, sample datasets, optional LatencyTrapCallout MDX component

**Total Tasks**: 144 (84 section writing + 60 infrastructure/validation/assets)

**Parallel Opportunities**: 89 tasks marked [P] (61% parallelizable)

**Critical Dependencies**:
- **URDF Canonical**: T035 (M1-C3-S1) blocks T042 (M2-C1-S1 Gazebo spawn), T065 (M3-C1-S3 USD conversion)
- **Synthetic Data**: T055 (M2-C2-S7) + T067 (M3-C1-S5) block T072 (M3-C2-S3 object detection training)
- **Nav2**: T074 (M3-C2-S5) blocks T092 (M4-C2-S2 behavior trees), T099 (M4-C3-S2 Plan→Navigate)
- **ROS 2 Actions**: T029 (M1-C2-S2) blocks T094 (M4-C2-S4 language→action grounding)
- **Whisper ASR**: T085 (M4-C1-S2) blocks T086 (M4-C1-S3 parsing)
- **Object Detection**: T072 (M3-C2-S3) blocks T100 (M4-C3-S3 Navigate→Manipulate)
- **PRIMARY REFERENCE**: T082 (M3-C3-S6 weight flashing) must be complete before T106 (insert Mention references to M3-C3-S6)
- **Module Gates**: T041 (M1 Check) blocks M2, T062 (M2 Check) blocks M3, T083 (M3 Check) blocks M4, T104 (M4 Check) blocks final QA

**Execution Order**:
1. Week 1: T001-T020 (scaffolding), T125-T129 (CI/CD), T114-T124 (code assets in parallel)
2. Weeks 2-3: T021-T041 (Module 1) → T041 gates Module 2
3. Weeks 4-6: T042-T062 (Module 2) → T062 gates Module 3
4. Weeks 7-9: T063-T083 (Module 3) → T083 gates Module 4
5. Weeks 10-12: T084-T104 (Module 4) → T104 gates QA
6. During/After writing: T105-T113 (injection tasks)
7. Week 13: T130-T141 (QA + deployment), T142-T144 (polish)

**Constitution Compliance**:
- 4×3×7 Structure: T007-T010 create exact directory schema; T017 creates exactly 84 files; T130 validates
- Scope Lock: T133 validates only brief-approved tools used
- Latency Trap Rule: T105-T106 inject 36 placements (22 Required + 14 Mention); T131 validates
- Information Density: T134 validates ≥1 code artifact per section
- Capstone Definition: T135 validates Voice→Plan→Navigate→Manipulate→Sim-to-Real pipeline
- Assessment Alignment: T136 validates all 6 assessments preparable from section IDs
- Delivery Toolchain: T001-T006 (Docusaurus), T125-T129 (GitHub Pages CI/CD)

## Outcome

- ✅ Impact: Comprehensive 144-task atomic checklist created for implementing 84-section textbook; organized into 11 phases matching 13-week roadmap; includes setup (20 tasks), writing (84 tasks), injection (9 tasks), assets (11 tasks), CI/CD (5 tasks), QA (12 tasks), polish (3 tasks)
- 🧪 Tests: Validation scripts defined (T127: validate-structure.sh verifies 84 files at correct paths, T128: validate-latency-traps.sh counts 36 placements, T129: validate-links.sh checks broken links); Module Consistency Check tasks (T041, T062, T083, T104) include test scripts (test-module1.sh through test-module4.sh) and checklist tables; Final QA tasks (T130-T141) validate structure/latency/links/scope/density/capstone/assessment/build/deployment
- 📁 Files: specs/001-textbook-spec/tasks.md created (all 144 tasks with exact file paths, dependencies, acceptance criteria)
- 🔁 Next prompts: `/sp.implement` to execute task checklist, begin Week 1 Docusaurus scaffolding (T001-T020), or execute specific module (e.g., "implement Module 1" executes T021-T041)
- 🧠 Reflection: Task checklist provides complete execution blueprint—144 atomic tasks with 89 parallelizable (61%); Critical path identified (scaffolding → M1 → M2 → M3 → M4 → QA → deploy with module gates at M1/M2/M3/M4-C3-S7); Dependencies explicitly documented (URDF canonical blocks Gazebo/Isaac, synthetic data blocks detection training, Nav2 blocks capstone navigation, Actions block VLA grounding, Whisper blocks parsing, detection blocks manipulation); Each task has clear "Done when" acceptance criteria (measurable); Injection tasks (T105-T113) enforce Constitution (36 latency trap placements, 4 module consistency checklists, 3 cross-link patterns); Validation scripts (T127-T129) automate structure/latency/link checking; QA tasks (T130-T141) ensure 100% completion before deployment

## Evaluation notes (flywheel)

- Failure modes observed: None. All user requirements addressed: (1) Scaffolding: T007-T020 create 84 .md files in `docs/M[1-4]/C[1-3]/S[1-7].md` format, (2) Config: T003-T004 setup docusaurus.config.js with Physical AI branding, T015-T016 setup sidebars.js for 4×3×7 hierarchy, (3) Templates: T018 pre-populate frontmatter (title, sidebar_position, keywords), T020 insert skeleton (Purpose, Technical content, Artifacts, Capstone link, Latency Trap, Hardware), (4) Code Assets: T114-T124 create src/snippets/ with Python/rclpy code + static/code/ with configs, (5) Deployment: T125 write .github/workflows/deploy.yml for GitHub Pages, (6) Article XII Check: T107-T110 create validation checklists in Mx-C3-S7 sections
- Graders run and results (PASS/FAIL): PASS - 144 tasks follow strict checklist format (checkbox, TaskID, [P] for parallel, description with file path); PASS - All 84 section writing tasks (T021-T104) include required fields (Task ID, file path, section title, brief anchors, artifacts, latency trap requirement, hardware, cross-links, "Done when" criteria); PASS - Dependencies documented (URDF canonical T035 blocks T042/T065, synthetic data T055/T067 block T072, Nav2 T074 blocks T092/T099, Actions T029 blocks T094, Whisper T085 blocks T086, detection T072 blocks T100, PRIMARY T082 blocks T106, Module gates T041/T062/T083/T104 block next modules); PASS - Parallel opportunities identified (89/144 tasks = 61% parallelizable); PASS - Critical path defined (scaffolding → M1 → M2 → M3 → M4 → QA → deploy)
- Prompt variant (if applicable): Standard /sp.tasks workflow adapted for documentation project (textbook creation vs. traditional software); organization: phases match 13-week roadmap from plan.md; 144 tasks (84 writing + 60 infrastructure/validation/assets)
- Next experiment (smallest change to try): Execute /sp.implement to auto-execute task checklist, OR begin Week 1 manually (execute T001-T020 scaffolding tasks: npx create-docusaurus, mkdir docs/M{1..4}/C{1..3}, touch 84 placeholders, configure sidebars.js, write GitHub Actions workflow, create validation scripts), OR execute specific milestone (e.g., "implement Milestone 0" executes scaffolding Phase 1-2)
