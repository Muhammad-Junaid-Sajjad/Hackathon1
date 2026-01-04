---
id: 26
title: "Elite Robotics Theme: Figure.ai & Agent Factory Refinement"
stage: "refactor"
date: 2026-01-04
surface: "agent"
model: "sonnet-4.5"
feature: "theme-overhaul"
branch: "main"
user: "nauman_sajjad"
command: "none"
labels: ["ui-ux", "figure.ai", "neon", "parallax", "light-mode-visibility"]
links:
  spec: "specs/001-textbook-spec/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - src/css/custom.css
  - src/pages/index.tsx
  - src/pages/index.module.css
  - docusaurus.config.ts
tests:
  - npm run build
---

## Prompt

Refine the UI to match Figure.ai/Agent Factory elite aesthetics. 1) Remove navbar logo/title/separators to make the site look like a "single whole". 2) Fix Light mode visibility issues where text/icons disappeared. 3) Replace the static book with a breathing, glowing white/blue robotic face that follows the cursor (mouse parallax). 4) Match Figure.ai typography (800 weight, tight tracking, neon glow).

## Response snapshot

Executed a high-fidelity visual overhaul of the landing page.

1. **Header UI**: Cleared navbar brand elements for a minimalist, integrated look.
2. **Light Mode visibility**: Mapped `--color-text-primary` to pure black in light mode and strengthened contrast for all text/icons to ensure readability.
3. **Figure.ai Hero Animation**: Implemented a `RoboticFace` component using React hooks. It features a breathing animation, glowing eyes, and a mouse-parallax effect where the face tilts and moves based on cursor coordinates.
4. **Neon Typography**: Applied `text-shadow` based neon pulses to the hero title and adjusted global heading weights to 800 for an elite technical feel.
5. **Structural Integrity**: Verified build and ensured all interaction logic (auto-greeting) remains functional within the new visual framework.

## Outcome

- ✅ Impact: The site now matches the "Elite" robotics aesthetic required. All visibility issues in Light mode are resolved.
- 🧪 Tests: `npm run build` passed (confirmed with static file generation).
- 📁 Files: Overhauled custom.css, index.tsx, index.module.css, and docusaurus.config.ts.

## Evaluation notes (flywheel)

- Failure modes observed: Initial broken anchors from Navbar; resolved by ensuring target IDs exist in the new component structure.
- Graders run and results (PASS/FAIL): PASS
- Next experiment: Add spatial audio cues for the robotic face interactions.
