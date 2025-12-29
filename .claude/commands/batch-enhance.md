# Batch Multi-Dimensional Enhancement Skill

Enhance multiple sections in parallel using specialized subagents with the multi-dimensional framework.

## Arguments
- `$ARGUMENTS` - Module/Chapter to enhance (e.g., "M2/C1" or "M2")

## Instructions

You are the **Batch Enhancement Coordinator**. Launch parallel subagents to enhance multiple sections using the multi-dimensional framework that serves all audience levels.

### Step 1: Identify Sections

List all sections in the specified module/chapter:
```bash
# Find all sections
ls docs/$ARGUMENTS/
# Or for full module
ls docs/$ARGUMENTS/C*/
```

Expected: S1.md through S7.md per chapter

### Step 2: Read Enhancement Intelligence Framework

Load the enhancement framework from:
`.specify/templates/m2-enhancement-intelligence.md`

This contains:
- Audience skill levels (Beginner → Architect)
- Industry perspective dimensions
- Agentic AI integration requirements
- Required enhancement elements

### Step 3: Launch Parallel Enhancement Agents

For each section, spawn a comprehensive enhancement agent:

```
Task: Multi-dimensional enhancement of section [path]
Subagent Type: general-purpose
Run in Background: true
Prompt: |
  You are a Multi-Dimensional Section Enhancer. Transform this section to serve:
  - 🌱 Beginners (clear definitions, step-by-step)
  - 🔧 Intermediate developers (implementation patterns)
  - ⚡ Elite engineers (optimization, production)
  - 🏗️ AI/Robotics architects (system design)
  - 🤖 Agentic AI engineers (autonomous integration)

  READ the section at: docs/[path].md
  READ the framework at: .specify/templates/m2-enhancement-intelligence.md

  ADD these elements:
  1. Prerequisites table with verification commands
  2. Tiered learning objectives (Beginner/Intermediate/Advanced/Architect)
  3. Key concepts table with "Why It Matters" column
  4. Skill-level pathway callouts (:::note, :::tip, :::caution)
  5. Industry spotlight boxes (2+ from different sectors)
  6. Elite insight callouts (2-3 production tips)
  7. Agentic AI consideration box with code example
  8. Architect's view callouts (system design)
  9. 4-tier practice exercises (Beginner → Architect)
  10. Enhanced troubleshooting with decision tree
  11. Summary table with key commands
  12. What's Next preview
  13. Further Reading with research papers

  PRESERVE:
  - All existing code examples
  - Technical accuracy
  - Section structure (enhance, don't reorganize)

  MDX RULES:
  - Escape < as &lt; in prose (not in code)
  - Use :::note, :::tip, :::warning, :::info, :::caution
  - Close all callout blocks with :::

  WRITE the enhanced section back to the same path.
```

### Parallel Processing Rules

**Batch Size**: Launch up to 3 sections in parallel
**Wait**: Complete batch before starting next
**Retry**: Failed sections get one retry
**Validate**: Check each section after enhancement

### Step 4: Quality Checklist (18-Point)

Score each enhanced section:

| # | Element | Points |
|---|---------|--------|
| 1 | Prerequisites table | 1 |
| 2 | Learning objectives (tiered) | 1 |
| 3 | Key concepts table | 1 |
| 4 | Skill-level pathways | 1 |
| 5 | Engaging introduction | 1 |
| 6 | Industry spotlight #1 | 1 |
| 7 | Industry spotlight #2 | 1 |
| 8 | Elite insight(s) | 1 |
| 9 | Agentic AI box | 1 |
| 10 | Architect's view | 1 |
| 11 | Exercise 1 (Beginner) | 1 |
| 12 | Exercise 2 (Intermediate) | 1 |
| 13 | Exercise 3 (Advanced) | 1 |
| 14 | Exercise 4 (Architect) | 1 |
| 15 | Troubleshooting guide | 1 |
| 16 | Summary table | 1 |
| 17 | What's Next | 1 |
| 18 | Code preserved | 1 |

**Pass Threshold**: 15/18 or higher

### Step 5: Build Verification

```bash
npm run build
```

Check for:
- MDX parsing errors
- Broken links
- Missing images

### Step 6: Report

```
╔══════════════════════════════════════════════════════════════╗
║          BATCH ENHANCEMENT REPORT: [Module/Chapter]           ║
╠══════════════════════════════════════════════════════════════╣
║ Section │ Status │ Score  │ Lines      │ Notes               ║
╠═════════╪════════╪════════╪════════════╪═════════════════════╣
║ S1      │ ✅     │ 18/18  │ 429→931    │ Reference impl      ║
║ S2      │ ✅     │ 17/18  │ 607→1024   │ Missing 1 exercise  ║
║ S3      │ ✅     │ 18/18  │ 832→1156   │                     ║
║ S4      │ ⚠️     │ 14/18  │ 674→845    │ Needs industry box  ║
║ S5      │ ✅     │ 16/18  │ 605→923    │                     ║
║ S6      │ ✅     │ 17/18  │ 759→1102   │                     ║
║ S7      │ ✅     │ 18/18  │ 600→987    │ Chapter capstone    ║
╠══════════════════════════════════════════════════════════════╣
║ TOTALS  │ 6/7 ✅ │ Avg 17 │ +2847 lines│                     ║
╠══════════════════════════════════════════════════════════════╣
║ Build Status: PASS ✅                                        ║
║ MDX Errors: 0                                                ║
║ Broken Links: 0                                              ║
╚══════════════════════════════════════════════════════════════╝

Sections Needing Revision:
- S4: Add 2 industry spotlight boxes

Enhancement Complete: [timestamp]
```

### Reusability

This skill can be reused for any module:
- `/batch-enhance M2/C1` → Enhance M2 Chapter 1
- `/batch-enhance M2/C2` → Enhance M2 Chapter 2
- `/batch-enhance M3` → Enhance all of Module 3
- `/batch-enhance M4/C3` → Enhance M4 Chapter 3

The enhancement framework at `.specify/templates/m2-enhancement-intelligence.md`
applies universally to all technical content in the textbook.
