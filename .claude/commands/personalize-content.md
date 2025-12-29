# Content Personalization Engine

Personalize textbook content based on user background, experience level, and learning goals.

## Arguments
- `$ARGUMENTS` - User profile JSON or section path for personalization rules

## Instructions

You are the **Content Personalization Specialist** for the Physical AI & Humanoid Robotics textbook. Your job is to adapt content to match each learner's background, making it more relevant and engaging.

### Personalization Philosophy

The textbook serves 5 audience levels. Personalization means:
1. **Emphasizing** content relevant to user's level
2. **Adding** contextual explanations where needed
3. **Highlighting** paths they should follow
4. **Connecting** to their stated goals
5. **Adjusting** depth based on preferred intensity

### Step 1: User Profile Schema

```typescript
interface UserBackground {
  // Experience levels
  programmingExperience: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'expert';
  roboticsExperience: 'none' | 'hobbyist' | 'student' | 'professional' | 'researcher';

  // Hardware context
  hardwareAccess: string[];  // ['jetson-nano', 'realsense', 'simulation-only', ...]

  // Goals
  learningGoals: string[];   // ['ros2', 'perception', 'manipulation', 'career', ...]

  // Content depth preference
  preferredDepth: 'beginner' | 'intermediate' | 'advanced';
}
```

### Step 2: Personalization Rules Engine

```python
"""
Content Personalization Engine

Adapts textbook content based on user background.
"""

from dataclasses import dataclass
from typing import List, Dict, Any
from enum import Enum

class ExperienceLevel(Enum):
    NONE = 0
    BEGINNER = 1
    INTERMEDIATE = 2
    ADVANCED = 3
    EXPERT = 4

@dataclass
class UserProfile:
    programming_exp: ExperienceLevel
    robotics_exp: ExperienceLevel
    hardware_access: List[str]
    learning_goals: List[str]
    preferred_depth: str

    @classmethod
    def from_dict(cls, data: dict) -> 'UserProfile':
        prog_map = {
            'none': ExperienceLevel.NONE,
            'beginner': ExperienceLevel.BEGINNER,
            'intermediate': ExperienceLevel.INTERMEDIATE,
            'advanced': ExperienceLevel.ADVANCED,
            'expert': ExperienceLevel.EXPERT
        }
        robot_map = {
            'none': ExperienceLevel.NONE,
            'hobbyist': ExperienceLevel.BEGINNER,
            'student': ExperienceLevel.INTERMEDIATE,
            'professional': ExperienceLevel.ADVANCED,
            'researcher': ExperienceLevel.EXPERT
        }
        return cls(
            programming_exp=prog_map.get(data.get('programming_experience', 'intermediate'), ExperienceLevel.INTERMEDIATE),
            robotics_exp=robot_map.get(data.get('robotics_experience', 'student'), ExperienceLevel.INTERMEDIATE),
            hardware_access=data.get('hardware_access', []),
            learning_goals=data.get('learning_goals', []),
            preferred_depth=data.get('preferred_depth', 'intermediate')
        )


class PersonalizationEngine:
    """Adapt content to user profile."""

    def __init__(self, profile: UserProfile):
        self.profile = profile

    def get_personalization_prompt(self, content: str, section_topic: str) -> str:
        """Generate GPT prompt for content personalization."""

        # Determine user's effective level
        avg_level = (self.profile.programming_exp.value + self.profile.robotics_exp.value) / 2

        if avg_level <= 1:
            level_name = "beginner"
            level_instructions = """
- Add extra explanations for technical terms
- Include more "why this matters" context
- Suggest skipping advanced sections on first read
- Highlight the :::note Beginner Path callouts
- Add encouraging tips like "Don't worry if this seems complex..."
"""
        elif avg_level <= 2.5:
            level_name = "intermediate"
            level_instructions = """
- Balance theory with practical implementation
- Highlight implementation patterns and best practices
- Point out common pitfalls to avoid
- Connect concepts to real projects they might build
"""
        else:
            level_name = "advanced/expert"
            level_instructions = """
- Emphasize production considerations
- Highlight Elite Insight and Architect's View callouts
- Point to optimization opportunities
- Connect to research papers and advanced topics
- Skip basic explanations they already know
"""

        # Hardware-specific adaptations
        hardware_notes = []
        if 'simulation-only' in self.profile.hardware_access:
            hardware_notes.append("- Emphasize simulation-based exercises (user has no physical hardware)")
        if 'jetson-nano' in self.profile.hardware_access or 'jetson-orin' in self.profile.hardware_access:
            hardware_notes.append("- Highlight Jetson-specific optimizations and deployment tips")
        if 'realsense' in self.profile.hardware_access:
            hardware_notes.append("- Emphasize Intel RealSense camera integration where relevant")
        if not hardware_notes:
            hardware_notes.append("- Provide balanced coverage of simulation and hardware options")

        # Goal-specific adaptations
        goal_notes = []
        goals = self.profile.learning_goals
        if 'ros2' in goals:
            goal_notes.append("- Emphasize ROS 2 concepts and patterns")
        if 'perception' in goals:
            goal_notes.append("- Highlight perception/sensing sections")
        if 'manipulation' in goals:
            goal_notes.append("- Focus on manipulation and control topics")
        if 'locomotion' in goals:
            goal_notes.append("- Emphasize locomotion and humanoid movement")
        if 'career' in goals:
            goal_notes.append("- Include industry insights and career tips")
        if 'research' in goals:
            goal_notes.append("- Point to research papers and cutting-edge topics")

        prompt = f"""You are a personalized learning assistant for a robotics textbook.

Adapt the following content for a learner with this profile:
- Experience Level: {level_name}
- Programming: {self.profile.programming_exp.name.lower()}
- Robotics: {self.profile.robotics_exp.name.lower()}
- Hardware: {', '.join(self.profile.hardware_access) or 'not specified'}
- Goals: {', '.join(self.profile.learning_goals) or 'general learning'}
- Preferred Depth: {self.profile.preferred_depth}

## Personalization Instructions:

### Level-Based Adaptations:
{level_instructions}

### Hardware-Specific Notes:
{chr(10).join(hardware_notes)}

### Goal-Specific Focus:
{chr(10).join(goal_notes) if goal_notes else '- Provide balanced coverage'}

## Rules:
1. PRESERVE all code examples exactly
2. PRESERVE all markdown formatting
3. ADD personalized callouts using this format:
   :::tip 💡 For You
   [Personalized tip based on their background]
   :::
4. HIGHLIGHT sections most relevant to their goals
5. ADD contextual bridges to their experience level
6. Keep the same overall structure
7. Do NOT remove any content, only enhance/emphasize

## Section Topic: {section_topic}

## Content to Personalize:
---
{content}
---

Return the personalized content with your adaptations integrated naturally."""

        return prompt

    def get_highlight_sections(self) -> List[str]:
        """Determine which section types to highlight for this user."""
        highlights = []

        # Level-based highlights
        avg_level = (self.profile.programming_exp.value + self.profile.robotics_exp.value) / 2
        if avg_level <= 1:
            highlights.extend([':::note Beginner Path', ':::note For Beginners', 'Prerequisites'])
        elif avg_level <= 2.5:
            highlights.extend([':::tip Intermediate Path', 'Implementation', 'Practice Exercises'])
        else:
            highlights.extend([':::tip Elite Insight', ':::info Architect\'s View', ':::warning Agentic AI'])

        # Goal-based highlights
        if 'perception' in self.profile.learning_goals:
            highlights.append('Sensor')
        if 'manipulation' in self.profile.learning_goals:
            highlights.append('Manipulation')
        if 'locomotion' in self.profile.learning_goals:
            highlights.append('Locomotion')

        return highlights

    def get_skip_suggestions(self) -> List[str]:
        """Suggest sections to skip on first read."""
        skips = []

        avg_level = (self.profile.programming_exp.value + self.profile.robotics_exp.value) / 2

        if avg_level <= 1:
            skips.extend([
                'Advanced optimization sections',
                'Mathematical derivations',
                'Production deployment details',
                'Research paper references'
            ])
        elif avg_level <= 2.5:
            skips.extend([
                'Basic concept definitions (if familiar)',
                'Some mathematical proofs'
            ])
        # Experts skip nothing

        return skips


async def personalize_content(content: str, user_profile: dict, section_topic: str) -> str:
    """
    Main entry point for content personalization.

    Args:
        content: Original markdown content
        user_profile: User background dictionary
        section_topic: Topic/title of the section

    Returns:
        Personalized markdown content
    """
    from openai import AsyncOpenAI
    import os

    client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))

    profile = UserProfile.from_dict(user_profile)
    engine = PersonalizationEngine(profile)

    prompt = engine.get_personalization_prompt(content, section_topic)

    response = await client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {
                "role": "system",
                "content": "You are an expert educational content personalizer. Your goal is to make technical content more accessible and relevant based on the learner's background."
            },
            {"role": "user", "content": prompt}
        ],
        temperature=0.4,
        max_tokens=8000
    )

    return response.choices[0].message.content
```

### Step 3: Personalization Callout Templates

For beginner users, add callouts like:
```markdown
:::tip 💡 For You (Beginner)
Since you're new to robotics, focus on understanding **what** this system does
before worrying about **how** it works internally. The key concept here is...
:::
```

For intermediate users:
```markdown
:::tip 💡 For You (Intermediate)
With your Python background, you'll find this pattern familiar to decorators.
The key difference in ROS 2 is...
:::
```

For advanced users:
```markdown
:::tip 💡 For You (Advanced)
Given your professional experience, pay special attention to the
production deployment patterns in the Elite Insight boxes below.
:::
```

### Step 4: API Integration

Add to `api/main.py`:

```python
@app.post("/api/personalize")
async def personalize_chapter(request: PersonalizeRequest):
    """
    Personalize chapter content for a specific user.
    """
    # Get user profile from database
    user_profile = await db.get_user_background(request.user_id)

    # Personalize content
    personalized = await personalize_content(
        content=request.chapter_content,
        user_profile=user_profile,
        section_topic=request.chapter_id
    )

    return {"personalized_content": personalized}
```

### Step 5: Validation

Verify personalized content:
- [ ] All code examples preserved exactly
- [ ] Markdown structure intact
- [ ] Personalization callouts added appropriately
- [ ] No content removed, only enhanced
- [ ] Tone matches user's level
- [ ] Hardware-specific tips included (if applicable)
- [ ] Goal-relevant sections highlighted

### Step 6: Report

```
╔══════════════════════════════════════════════════════════════╗
║           PERSONALIZATION REPORT                              ║
╠══════════════════════════════════════════════════════════════╣
║ User Profile:                                                 ║
║ ├─ Programming: [level]                                       ║
║ ├─ Robotics: [level]                                          ║
║ ├─ Hardware: [list]                                           ║
║ ├─ Goals: [list]                                              ║
║ └─ Preferred Depth: [level]                                   ║
╠══════════════════════════════════════════════════════════════╣
║ Content Adaptations:                                          ║
║ ├─ Personalization Level: [beginner/intermediate/advanced]    ║
║ ├─ Callouts Added: [count]                                    ║
║ ├─ Sections Highlighted: [list]                               ║
║ └─ Skip Suggestions: [list]                                   ║
╠══════════════════════════════════════════════════════════════╣
║ Quality Checks:                                               ║
║ ├─ Code Preserved: ✅                                         ║
║ ├─ Structure Intact: ✅                                       ║
║ ├─ Tone Appropriate: ✅                                       ║
║ └─ Goals Addressed: ✅                                        ║
╚══════════════════════════════════════════════════════════════╝

Sample Personalization:
"Since you're learning ROS 2 with a Python background, this section on
nodes will feel familiar. Think of ROS 2 nodes like Python modules..."
```

## Reusability

- `/personalize-content {"programming_experience": "beginner", ...}` - Test with profile
- `/personalize-content M2/C1/S3` - View personalization rules for section
- Called by ChapterTools component for logged-in users
