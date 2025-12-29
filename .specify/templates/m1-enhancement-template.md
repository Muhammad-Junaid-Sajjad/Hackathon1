# M1 Enhancement Template (ROS 2 / Robotic Nervous System)

Use this template to enhance Module 1 sections to meet the Pedagogical Excellence Standard.

## Required Structure

```markdown
---
id: m1-cX-sY
title: [Title]
sidebar_position: Y
keywords: ['keyword1', 'keyword2', ...]
---

# [Title]

## Prerequisites

Before starting this section, you should have:
- Completed M1-C1-S1 (Workstation Setup with Ubuntu 22.04)
- [Other relevant prerequisites from earlier sections]
- Basic understanding of [relevant concepts]
- [Hardware/software requirements]

## Learning Objectives

By the end of this section, you will be able to:
- **Define** [key concept] and its role in robotics
- **Explain** how [concept] enables [capability]
- **Implement** [practical skill] using ROS 2
- **Configure** [tool/component] for [purpose]
- **Verify** [expected outcome]

## Key Concepts

| Term | Definition |
|------|------------|
| **[Term 1]** | [Clear one-sentence definition] |
| **[Term 2]** | [Clear one-sentence definition] |
| **[Term 3]** | [Clear one-sentence definition] |
| **[Term 4]** | [Clear one-sentence definition] |
| **[Term 5]** | [Clear one-sentence definition] |

---

## Introduction: [Engaging Hook]

[2-3 paragraphs that:
- Start with a relatable scenario or question
- Explain why this topic matters for humanoid robotics
- Preview what they'll learn and build]

:::tip Why This Matters
[Brief callout explaining practical importance - connect to real robotics companies/applications]
:::

---

## What Is [Main Concept]?

### Definition

**[Concept]** is [one clear sentence definition].

[1-2 paragraphs expanding on the definition with context]

### Why Do We Need [Concept]?

**Without [Concept]:**
- [Problem 1]
- [Problem 2]
- [Problem 3]

**With [Concept]:**
- [Benefit 1]
- [Benefit 2]
- [Benefit 3]

### How Does [Concept] Work?

[Step-by-step explanation]

```
┌─────────────────────────────────────────────────────────────┐
│                    [CONCEPT] ARCHITECTURE                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌───────────┐    ┌───────────┐    ┌───────────┐           │
│  │ Component │───▶│ Component │───▶│ Component │           │
│  │     A     │    │     B     │    │     C     │           │
│  └───────────┘    └───────────┘    └───────────┘           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Real-World Example

**Industry Application**: [How this is used at Tesla/Boston Dynamics/etc.]

**Relatable Analogy**: Think of [concept] like [everyday analogy].

---

## Implementation

[Keep all existing code examples - they are good]

### [Implementation Step 1]

:::warning Common Mistake
[Describe a common error and how to fix it]
:::

### [Implementation Step 2]

**Expected Output:**
```
[What the reader should see]
```

---

## Connection to Capstone

This section directly supports the capstone project:

| Capstone Component | How This Section Helps |
|-------------------|------------------------|
| **Voice Command** | [Specific connection or N/A] |
| **Planning** | [Specific connection or N/A] |
| **Navigation** | [Specific connection or N/A] |
| **Vision** | [Specific connection or N/A] |
| **Manipulation** | [Specific connection or N/A] |

```
┌─────────────────────────────────────────────────────────────┐
│                    CAPSTONE PIPELINE                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Voice ──▶ Plan ──▶ Navigate ──▶ Vision ──▶ Manipulate     │
│              │                                              │
│         [THIS SECTION]                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Summary

In this section, you learned:
- [Key takeaway 1]
- [Key takeaway 2]
- [Key takeaway 3]

**Key Commands to Remember:**
```bash
# [Command 1 purpose]
[command]

# [Command 2 purpose]
[command]
```

---

## Practice Exercises

### Exercise 1: Basic - [Title]
**Objective:** [What skill this practices]
**Time:** ~10 minutes

[Clear instructions]

**Expected Result:** [What success looks like]

<details>
<summary>Hint</summary>
[Helpful hint]
</details>

---

### Exercise 2: Intermediate - [Title]
**Objective:** [Combines multiple concepts]
**Time:** ~20 minutes

[Instructions]

**Success Criteria:**
- [ ] [Criterion 1]
- [ ] [Criterion 2]

---

### Exercise 3: Challenge - [Title]
**Objective:** [Extends beyond section content]
**Time:** ~30+ minutes

[Open-ended challenge]

**Bonus:** [Extra challenge]

---

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| [Error 1] | [Why it happens] | [How to fix] |
| [Error 2] | [Why it happens] | [How to fix] |
| [Error 3] | [Why it happens] | [How to fix] |
| [Error 4] | [Why it happens] | [How to fix] |

---

## What's Next?

In the next section, **[Next Section Title]**, you will learn:
- [Preview 1]
- [Preview 2]
- [Preview 3]

This will enable you to [exciting capability].

---

## Further Reading

- [Official documentation link]
- [Tutorial link]
- [Research paper or advanced topic]

:::info Industry Insight
[Fun fact about how this is used in real robotics companies]
:::
```

## M1-Specific Content Guidelines

### Chapter 1: Hardware & Setup
- Focus on workstation, Jetson, physical setup
- Emphasize latency requirements
- Connect to edge vs workstation architecture

### Chapter 2: ROS 2 Communication
- Focus on nodes, topics, services, actions
- Emphasize pub/sub patterns
- Connect to real-time control requirements

### Chapter 3: Robot Description
- Focus on URDF, transforms, visualization
- Emphasize kinematics and geometry
- Connect to IK/FK requirements

## MDX Rules (CRITICAL)
- Escape `<` in prose as `&lt;`
- Use backticks for variables like `variable_name`
- Keep code blocks unchanged (safe in MDX)
- Use `:::tip`, `:::warning`, `:::info` for callouts
