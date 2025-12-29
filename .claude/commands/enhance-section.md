# Section Enhancement Orchestrator

Orchestrate multi-dimensional section enhancement using specialized subagents.

## Arguments
- `$ARGUMENTS` - Section path (e.g., "M2/C1/S2" or "docs/M2/C1/S2.md")

## Instructions

You are the **Master Enhancement Orchestrator**. Your job is to coordinate specialized subagents to enhance a section with multi-dimensional content for all audience levels.

### Step 1: Analyze Current Section

Read the target section and extract:
1. Current structure and content
2. Technical topic and domain
3. Existing code examples (preserve these!)
4. What's missing from the enhancement framework

Reference the intelligence framework: `.specify/templates/m2-enhancement-intelligence.md`

### Step 2: Launch Specialized Subagents

Launch these subagents IN PARALLEL for efficiency:

#### Subagent 1: Pedagogical Enhancement Agent
```
Task: Add pedagogical elements to section [path]
Subagent Type: general-purpose
Prompt: |
  You are a Pedagogical Enhancement Specialist. Add these elements to the section:

  1. PREREQUISITES TABLE
  - List 3-5 prerequisites with verification commands
  - Reference prior sections (M1-C1-S1 format)

  2. LEARNING OBJECTIVES (Tiered)
  - 2 Beginner objectives (Define/Identify)
  - 2 Intermediate objectives (Implement/Configure)
  - 2 Advanced objectives (Optimize/Architect)

  3. KEY CONCEPTS TABLE
  - 5-7 terms with Definition + "Why It Matters"

  4. SKILL-LEVEL PATHWAYS
  - :::note Beginner Path - what to focus on, what to skip
  - :::tip Intermediate Path - deep dive areas
  - :::caution Advanced Path - production considerations

  5. SUMMARY TABLE
  - Key concept | Key takeaway format
  - Key commands to remember

  RULES:
  - Preserve ALL existing code examples
  - Use MDX callout syntax (:::note, :::tip, etc.)
  - Match the technical level of existing content
```

#### Subagent 2: Industry Perspective Agent
```
Task: Add industry perspectives to section [path]
Subagent Type: general-purpose
Prompt: |
  You are an Industry Robotics Expert with 20+ years across manufacturing, healthcare, logistics, and research. Add:

  1. INDUSTRY SPOTLIGHT BOXES (2 minimum)
  Format:
  :::info Industry Spotlight: [Sector] ([Companies])
  **How [sector] uses this:**
  - [2-3 specific applications]

  **Key metrics they care about:**
  - [Metric 1]: [Value/Range]
  - [Metric 2]: [Value/Range]

  **Lesson learned:** "[Quote or insight]"
  :::

  2. ELITE INSIGHTS (2-3)
  Format:
  :::tip Elite Insight: [Topic]
  [Production-grade insight about performance, scaling, security, or edge cases]
  :::

  3. REAL-WORLD EXAMPLES
  - Reference actual companies: Tesla, Boston Dynamics, Amazon, NASA, etc.
  - Include real specifications and metrics where possible

  SECTORS TO CONSIDER:
  - Manufacturing (Tesla, BMW, FANUC)
  - Healthcare (Intuitive Surgical, Medtronic)
  - Logistics (Amazon Robotics, Locus)
  - Research (MIT, Stanford, CMU)
  - Consumer (iRobot, Boston Dynamics)
```

#### Subagent 3: Agentic AI Integration Agent
```
Task: Add agentic AI perspective to section [path]
Subagent Type: general-purpose
Prompt: |
  You are an Agentic AI Systems Architect specializing in autonomous robots. Add:

  1. AGENTIC AI CONSIDERATION BOX
  Format:
  :::warning 🤖 Agentic AI Consideration
  **For autonomous systems:**

  **Perception**: How this feeds into world models
  **Planning**: How AI agents use this for decisions
  **Action**: How to command this programmatically
  **Learning**: What data to log for training

  **Integration Pattern:**
  ```python
  # Example code showing AI/LLM integration
  class AgentInterface:
      def use_this_capability(self, goal):
          # Pattern for autonomous use
  ```

  **Safety Constraints:**
  - [What must never happen autonomously]
  - [Human-in-the-loop requirements]
  :::

  2. ARCHITECT'S VIEW BOXES (1-2)
  Format:
  :::info Architect's View: [Topic]
  [System design consideration for large-scale or production systems]
  - Scalability patterns
  - Integration with AI planning systems
  - Cross-cutting concerns
  :::

  FOCUS AREAS:
  - How LLMs can interface with this system
  - Monte Carlo planning using simulation
  - Safety validation for autonomous operation
  - Data collection for continuous learning
```

#### Subagent 4: Exercise Generator Agent
```
Task: Generate tiered exercises for section [path]
Subagent Type: general-purpose
Prompt: |
  You are an Expert Robotics Educator. Generate 4 practice exercises:

  ### Exercise 1: Foundation (Beginner)
  **Objective:** [Build confidence with basics]
  **Time:** ~15 minutes
  **Skills Practiced:** [List]

  [Clear step-by-step instructions]

  **Success Criteria:**
  - [ ] [Observable outcome 1]
  - [ ] [Observable outcome 2]

  <details>
  <summary>💡 Hint</summary>
  [Helpful hint]
  </details>

  ---

  ### Exercise 2: Integration (Intermediate)
  **Objective:** [Combine multiple concepts]
  **Time:** ~30 minutes
  [Instructions with some decisions left to student]

  ---

  ### Exercise 3: Production Challenge (Advanced)
  **Objective:** [Real-world scenario]
  **Time:** ~60+ minutes
  **Scenario:** [Realistic business/technical scenario]
  [Complex requirements with constraints]

  ---

  ### Exercise 4: Architect's Design (Expert)
  **Objective:** [System design exercise]
  **Time:** ~2+ hours
  [Open-ended design challenge involving AI integration]

  RULES:
  - Exercises must build on section content
  - Include success criteria for all
  - Add hints for exercises 1-2
  - Make exercise 4 relevant to agentic AI
```

### Step 3: Merge Results

Combine subagent outputs into the section following this structure:

```markdown
---
[frontmatter - preserve existing]
---

# [Title]

## Prerequisites
[From Pedagogical Agent]

## Learning Objectives
[From Pedagogical Agent]

## Key Concepts
[From Pedagogical Agent]

## Skill-Level Pathways
[From Pedagogical Agent]

## Introduction
[Enhanced with why-it-matters from existing + new insights]

[EXISTING CONTENT - preserved and enhanced with callouts]

## Industry Perspectives
[From Industry Agent - integrate as callout boxes throughout]

## Agentic AI Integration
[From Agentic AI Agent]

## Summary
[From Pedagogical Agent]

## Practice Exercises
[From Exercise Generator]

## Troubleshooting Guide
[Enhanced or created]

## What's Next?
[Preview next section]

## Further Reading
[Add relevant links]
```

### Step 4: Validate

Check the enhanced section:
- [ ] All existing code preserved
- [ ] No MDX syntax errors (escape < as &lt; in prose)
- [ ] All callout boxes properly closed
- [ ] Prerequisites reference correct prior sections
- [ ] Exercises are achievable with section knowledge
- [ ] Industry examples are accurate

### Step 5: Report

Output summary:
```
Enhanced: [Section Path]
Lines: [Before] → [After]
Added Elements:
- Prerequisites: ✅
- Learning Objectives: ✅
- Key Concepts: ✅
- Skill Pathways: ✅
- Industry Spotlights: [count]
- Agentic AI Box: ✅
- Elite Insights: [count]
- Exercises: 4
- Troubleshooting: ✅
```
