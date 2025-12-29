# Add Examples Skill

Add working code examples with expected output to a section.

## Arguments
- `$ARGUMENTS` - Section path (e.g., "M1/C2/S4")

## Instructions

For each concept, add:

### 1. Simple Example
Minimal code showing concept in isolation:
```markdown
### Simple Example

```python
# File: simple_example.py
# Purpose: Demonstrate [concept] in isolation

[10-20 lines of minimal working code]
```

**Expected Output:**
```
[Exact output the reader should see]
```
```

### 2. Real-World Example
How it's used in actual robotics:
```markdown
### Real-World Application

In production robotics systems, [concept] is used for:

```python
# File: robotics_example.py
# Context: [Real robotics scenario]

[Working code with comments]
```

**What This Does:**
- [Explanation of each key line]
```

### 3. Common Mistakes
```markdown
:::warning Common Mistake
**Wrong:**
```python
[Incorrect code]
```

**Problem:** [Why this fails]

**Correct:**
```python
[Fixed code]
```
:::
```

### Guidelines
- All code must be runnable (no pseudocode)
- Show EXACT expected output
- Include file names for reference
- Add comments explaining key lines
- Test code mentally for errors
