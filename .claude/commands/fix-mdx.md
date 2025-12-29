# Fix MDX Errors Skill

Fix MDX parsing errors in textbook sections.

## Arguments
- `$ARGUMENTS` - Section path or "all" to scan all sections

## Instructions

Scan and fix common MDX errors that break Docusaurus builds.

### Common MDX Issues

#### 1. Curly Braces `{variable}`
**Problem:** MDX interprets `{i}` as JSX expression
**Solution:** Replace with HTML entity or remove braces

```markdown
# Wrong (causes ReferenceError)
| Symbol | Description |
| a_{i} | Distance from z_{i} to z_{i+1} |

# Correct
| Symbol | Description |
| a&#95;i | Distance from z&#95;i to z&#95;i+1 |
```

#### 2. Comparison Operators
**Problem:** `<` and `>` interpreted as JSX tags
**Solution:** Use HTML entities in prose (not code blocks)

```markdown
# Wrong (in table or prose)
| Latency | < 10ms |

# Correct
| Latency | &lt; 10ms |
```

#### 3. Generic Types
**Problem:** `List<T>` looks like JSX
**Solution:** Escape or use code formatting

```markdown
# Wrong
The function returns List<String>

# Correct
The function returns `List<String>`
# or
The function returns List&lt;String&gt;
```

### Scanning Process
1. Read section file
2. Search for patterns outside code blocks:
   - `{[a-z]}` - single letter in braces
   - `{[a-z]+}` - words in braces
   - `< ` or ` >` - comparison operators
   - `<[A-Z]` - generic type syntax
3. Apply fixes
4. Verify build passes

### Safe Zones (Don't Modify)
- Inside triple-backtick code blocks
- Inside inline code backticks
- URLs

### Output
```
MDX Fix Report: [section]
=========================
Found [N] issues:
- Line [X]: {i} → &#95;i
- Line [Y]: < 10ms → &lt; 10ms

Fixed: [N] issues
Build: [PASS/FAIL]
```
