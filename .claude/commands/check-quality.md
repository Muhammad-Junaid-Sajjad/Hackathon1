# Check Section Quality Skill

Audit a textbook section against the Pedagogical Excellence Standard.

## Arguments
- `$ARGUMENTS` - Section path (e.g., "M1/C1/S2") or "all" for full audit

## Instructions

Read the section and score against these criteria:

### Quality Checklist (Score 0-1 each)
| Element | Present? | Score |
|---------|----------|-------|
| Prerequisites | | |
| Learning Objectives (3+) | | |
| Key Concepts table | | |
| ASCII diagram(s) | | |
| Theory sections (Def/Why/How) | | |
| Real-world examples | | |
| Code with expected output | | |
| Connection to Capstone | | |
| Practice Exercises (3) | | |
| Troubleshooting table | | |
| What's Next preview | | |
| Engagement callouts | | |

### Content Balance Check
- Theory: ___% (target: 30-40%)
- Practice: ___% (target: 30-40%)
- Visuals: ___% (target: 20-30%)

### Output Format
```
Section: [path]
Score: [X/12] ([percentage]%)
Status: [PASS/NEEDS_WORK/FAIL]

Missing Elements:
- [element 1]
- [element 2]

Recommendations:
- [specific improvement 1]
- [specific improvement 2]
```

PASS = 10+/12, NEEDS_WORK = 7-9/12, FAIL = <7/12
