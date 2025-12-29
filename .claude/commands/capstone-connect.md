# Connect to Capstone Skill

Add explicit capstone project connections to a section.

## Arguments
- `$ARGUMENTS` - Section path (e.g., "M2/C1/S3")

## Instructions

Every section must show how its content enables the capstone project.

### Capstone Pipeline
```
Voice Command → Cognitive Planning → Navigation → Vision → Manipulation
   (Whisper)      (LLM Planning)     (VSLAM)    (YOLO)   (Grasp Control)
```

### Add Connection Table
```markdown
## Connection to Capstone

This section directly supports the capstone project:

| Capstone Stage | How This Section Helps |
|----------------|------------------------|
| **Voice Command** | [Specific connection or "N/A"] |
| **Cognitive Planning** | [Specific connection or "N/A"] |
| **Navigation** | [Specific connection or "N/A"] |
| **Vision** | [Specific connection or "N/A"] |
| **Manipulation** | [Specific connection or "N/A"] |
```

### Add Pipeline Diagram
```markdown
```
┌─────────────────────────────────────────────────────────────┐
│                    CAPSTONE PIPELINE                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Voice ───▶ Plan ───▶ Navigate ───▶ Vision ───▶ Manipulate │
│                          │                                  │
│                    ┌─────┴─────┐                            │
│                    │ THIS      │                            │
│                    │ SECTION   │                            │
│                    └───────────┘                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```
[Highlight which stage(s) this section supports]
```

### Connection Examples by Module
- **M1 (ROS 2)**: Communication backbone for all stages
- **M2 (Simulation)**: Testing environment before physical deployment
- **M3 (Isaac)**: Perception and navigation acceleration
- **M4 (VLA)**: Voice-to-action end-to-end integration

### Guidelines
- Be SPECIFIC about the connection
- Show code/concepts that directly apply
- Explain what would break without this section
- Reference specific capstone requirements
