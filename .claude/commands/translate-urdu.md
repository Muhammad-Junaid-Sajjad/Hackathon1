# Urdu Translation Generator

Generate high-quality Urdu translations for textbook content using OpenAI.

## Arguments
- `$ARGUMENTS` - Section path (e.g., "M2/C1/S3") or content snippet for translation

## Instructions

You are the **Urdu Translation Specialist** for the Physical AI & Humanoid Robotics textbook. Your job is to create accurate, readable Urdu translations that preserve technical meaning while being culturally appropriate for Pakistani/Urdu-speaking audiences.

### Translation Philosophy

1. **Technical Terms**: Keep in English with Urdu transliteration in parentheses
   - Example: "ROS 2 (آر او ایس ٹو)"
   - Example: "Neural Network (نیورل نیٹ ورک)"

2. **Code & Commands**: Never translate - keep original
   ```bash
   ros2 run package_name node_name
   ```

3. **Proper Nouns**: Keep original with Urdu script if common
   - "Gazebo" → "Gazebo (گزیبو)"
   - "Python" → "Python (پائتھون)"

4. **Flow**: Use natural Urdu sentence structure (SOV)
   - English: "The robot moves forward"
   - Urdu: "روبوٹ آگے کی طرف حرکت کرتا ہے"

### Step 1: Extract Content

Read the target section and identify translatable content:

```python
# Content types to translate
TRANSLATE = [
    'prose_paragraphs',
    'section_headers',
    'callout_boxes_text',
    'exercise_instructions',
    'table_headers',
    'figure_captions'
]

# Content to preserve (never translate)
PRESERVE = [
    'code_blocks',
    'terminal_commands',
    'file_paths',
    'URLs',
    'mathematical_formulas',
    'variable_names'
]
```

### Step 2: Translation Prompt Template

For each translatable chunk, use this prompt structure:

```
You are an expert Urdu translator specializing in robotics and AI education.
Translate the following English text to Urdu, following these rules:

1. KEEP IN ENGLISH (with Urdu transliteration):
   - Technical terms: ROS 2, Gazebo, URDF, TF2, etc.
   - Programming keywords: class, function, import
   - Acronyms: AI, ML, GPU, API

2. TRANSLATE NATURALLY:
   - Use conversational Pakistani Urdu
   - Use دیکھیں (see) instead of ملاحظہ فرمائیں (formal)
   - Address reader as آپ (you - respectful)
   - Use active voice where possible

3. PRESERVE EXACTLY:
   - Code blocks (```...```)
   - File paths (/path/to/file)
   - Commands (ros2 run ...)
   - Mathematical notation

4. FORMATTING:
   - Keep markdown structure
   - Preserve line breaks
   - Keep bullet points and numbers

Text to translate:
---
{content}
---

Provide ONLY the Urdu translation, no explanations.
```

### Step 3: Generate Translation API

Create `api/translation.py`:

```python
"""
Urdu Translation Service for Physical AI Textbook

Uses OpenAI for high-quality technical translation.
"""

import os
import re
from typing import List, Tuple
from openai import AsyncOpenAI

client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Patterns to preserve (not translate)
PRESERVE_PATTERNS = [
    (r'```[\s\S]*?```', 'CODE_BLOCK'),           # Code blocks
    (r'`[^`]+`', 'INLINE_CODE'),                  # Inline code
    (r'!\[.*?\]\(.*?\)', 'IMAGE'),                # Images
    (r'\[.*?\]\(.*?\)', 'LINK'),                  # Links
    (r'https?://\S+', 'URL'),                     # URLs
    (r'/[\w/.-]+', 'FILEPATH'),                   # File paths
    (r'\$.*?\$', 'MATH'),                         # Math expressions
]

# Technical terms to keep in English with transliteration
TECHNICAL_GLOSSARY = {
    'ROS 2': 'ROS 2 (آر او ایس ٹو)',
    'Gazebo': 'Gazebo (گزیبو)',
    'Python': 'Python (پائتھون)',
    'node': 'node (نوڈ)',
    'topic': 'topic (ٹاپک)',
    'service': 'service (سروس)',
    'action': 'action (ایکشن)',
    'package': 'package (پیکیج)',
    'workspace': 'workspace (ورک سپیس)',
    'URDF': 'URDF (یو آر ڈی ایف)',
    'TF2': 'TF2 (ٹی ایف ٹو)',
    'SLAM': 'SLAM (سلیم)',
    'LiDAR': 'LiDAR (لائیڈار)',
    'IMU': 'IMU (آئی ایم یو)',
    'GPU': 'GPU (جی پی یو)',
    'API': 'API (اے پی آئی)',
    'AI': 'AI (اے آئی)',
    'ML': 'ML (ایم ایل)',
    'neural network': 'neural network (نیورل نیٹ ورک)',
    'deep learning': 'deep learning (ڈیپ لرننگ)',
    'simulation': 'simulation (سمولیشن)',
    'robot': 'روبوٹ',
    'sensor': 'سینسر',
    'motor': 'موٹر',
    'controller': 'کنٹرولر',
}


def preserve_protected_content(text: str) -> Tuple[str, List[Tuple[str, str]]]:
    """Replace protected content with placeholders."""
    preserved = []
    for pattern, name in PRESERVE_PATTERNS:
        matches = re.findall(pattern, text)
        for i, match in enumerate(matches):
            placeholder = f'___{name}_{i}___'
            text = text.replace(match, placeholder, 1)
            preserved.append((placeholder, match))
    return text, preserved


def restore_protected_content(text: str, preserved: List[Tuple[str, str]]) -> str:
    """Restore protected content from placeholders."""
    for placeholder, original in preserved:
        text = text.replace(placeholder, original)
    return text


async def translate_to_urdu(content: str, chapter_context: str = "") -> str:
    """
    Translate content to Urdu using OpenAI.

    Args:
        content: English markdown content
        chapter_context: Optional context about the chapter topic

    Returns:
        Urdu translated content with preserved code/formatting
    """
    # Step 1: Preserve protected content
    text, preserved = preserve_protected_content(content)

    # Step 2: Build translation prompt
    prompt = f"""You are an expert Urdu translator specializing in robotics and AI education for Pakistani audiences.

Translate the following English text to Urdu, following these rules:

## TECHNICAL TERMS (keep in English with Urdu transliteration):
- ROS 2, Gazebo, URDF, TF2, Python, etc.
- Programming terms: class, function, import, variable
- Acronyms: AI, ML, GPU, API, SLAM

## TRANSLATE NATURALLY:
- Use conversational Pakistani Urdu (not formal)
- Address reader as آپ (respectful you)
- Use active voice where possible
- Keep explanations clear and educational

## PRESERVE EXACTLY (marked with ___ placeholders):
- Everything between ___CODE_BLOCK_*___ markers
- Everything between ___INLINE_CODE_*___ markers
- All other ___*___ placeholders

## FORMATTING:
- Keep all markdown structure (##, -, *, etc.)
- Preserve line breaks and spacing
- Keep bullet points and numbered lists

{"Context: This is from a chapter about " + chapter_context if chapter_context else ""}

---
TEXT TO TRANSLATE:
{text}
---

Provide ONLY the Urdu translation. Preserve all placeholders exactly as shown."""

    # Step 3: Call OpenAI
    response = await client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {
                "role": "system",
                "content": "You are a professional Urdu translator specializing in technical robotics education. Your translations are accurate, natural, and accessible to Pakistani learners."
            },
            {"role": "user", "content": prompt}
        ],
        temperature=0.3,  # Lower for consistency
        max_tokens=4000
    )

    translated = response.choices[0].message.content

    # Step 4: Restore protected content
    result = restore_protected_content(translated, preserved)

    # Step 5: Apply glossary substitutions
    for eng, urdu in TECHNICAL_GLOSSARY.items():
        # Only replace if not already transliterated
        pattern = rf'\b{re.escape(eng)}\b(?!\s*\([^)]*[اآبپتٹثجچحخدڈذرڑزژسشصضطظعغفقکگلمنوہھیے])'
        result = re.sub(pattern, urdu, result, flags=re.IGNORECASE)

    return result


async def translate_section(section_path: str) -> dict:
    """
    Translate an entire section.

    Returns:
        {
            'original': str,
            'translated': str,
            'word_count': int,
            'status': 'success' | 'error',
            'error': str | None
        }
    """
    from pathlib import Path

    filepath = Path(f'docs/{section_path}.md')
    if not filepath.exists():
        return {
            'status': 'error',
            'error': f'Section not found: {filepath}'
        }

    content = filepath.read_text(encoding='utf-8')

    # Extract title from frontmatter for context
    import yaml
    if content.startswith('---'):
        parts = content.split('---', 2)
        if len(parts) >= 3:
            try:
                meta = yaml.safe_load(parts[1])
                context = meta.get('title', section_path)
            except:
                context = section_path
    else:
        context = section_path

    try:
        translated = await translate_to_urdu(content, context)
        return {
            'original': content,
            'translated': translated,
            'word_count': len(content.split()),
            'status': 'success',
            'error': None
        }
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e)
        }
```

### Step 4: RTL Styling

Ensure CSS supports RTL for Urdu content:

```css
/* In styles.module.css */
.urduContent {
  direction: rtl;
  text-align: right;
  font-family: 'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', 'Mehr Nastaliq', serif;
  line-height: 2;
  font-size: 1.1em;
}

.urduContent code,
.urduContent pre {
  direction: ltr;
  text-align: left;
  font-family: 'Fira Code', monospace;
}

.urduContent h1, .urduContent h2, .urduContent h3 {
  font-family: 'Noto Nastaliq Urdu', serif;
}
```

### Step 5: Validation Checklist

After translation, verify:
- [ ] All code blocks preserved unchanged
- [ ] Technical terms have Urdu transliteration
- [ ] RTL displays correctly
- [ ] Links and images work
- [ ] Mathematical formulas preserved
- [ ] Markdown structure intact
- [ ] No placeholder markers visible

### Step 6: Report

```
╔══════════════════════════════════════════════════════════════╗
║              URDU TRANSLATION REPORT                          ║
╠══════════════════════════════════════════════════════════════╣
║ Section: [section_path]                                       ║
║ Original Words: [count]                                       ║
║ Translation Status: ✅ Success                                ║
╠══════════════════════════════════════════════════════════════╣
║ Quality Checks:                                               ║
║ ├─ Code Blocks Preserved: ✅                                  ║
║ ├─ Technical Terms Handled: ✅                                ║
║ ├─ Markdown Structure: ✅                                     ║
║ ├─ RTL Formatting: ✅                                         ║
║ └─ Links & Images: ✅                                         ║
╠══════════════════════════════════════════════════════════════╣
║ Technical Terms Transliterated:                               ║
║ - ROS 2 → ROS 2 (آر او ایس ٹو)                               ║
║ - Gazebo → Gazebo (گزیبو)                                    ║
║ - [... more terms ...]                                        ║
╚══════════════════════════════════════════════════════════════╝

Sample Translation:
English: "This section introduces ROS 2 nodes..."
Urdu: "اس سیکشن میں ROS 2 (آر او ایس ٹو) نوڈز کا تعارف..."
```

## Reusability

- `/translate-urdu M2/C1/S3` - Translate specific section
- `/translate-urdu "text to translate"` - Translate snippet
- Called by ChapterTools component for on-demand translation
