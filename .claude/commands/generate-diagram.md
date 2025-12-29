# Generate Diagram Skill

Generate ASCII diagrams for robotics textbook content.

## Arguments
- `$ARGUMENTS` - Diagram description (e.g., "ROS 2 pub-sub architecture")

## Instructions

Create clear ASCII diagrams using box-drawing characters: ┌ ┐ └ ┘ │ ─ ├ ┤ ┬ ┴ ┼ ▶ ◀ ▲ ▼

### Diagram Types
1. **Architecture**: System components with data flow arrows
2. **Flowchart**: Process steps with decision diamonds
3. **Hierarchy**: Tree structures (URDF, TF trees)
4. **Sequence**: Time-ordered message passing
5. **Comparison**: Side-by-side feature tables

### Guidelines
- Max 75 characters wide
- Label all components
- Show data flow direction with arrows
- Add brief description below diagram

Return diagram wrapped in triple backticks.
