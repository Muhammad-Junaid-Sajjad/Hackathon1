---
id: 25
title: "Interactive AI Guide: Hybrid Engagement"
stage: "refactor"
date: 2026-01-04
surface: "agent"
model: "sonnet-4.5"
feature: "interactive-hero"
branch: "main"
user: "nauman_sajjad"
command: "none"
labels: ["interactivity", "react", "animation", "ux"]
links:
  spec: "specs/001-textbook-spec/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
  - src/pages/index.tsx
  - src/pages/index.module.css
tests:
  - npm run build
---

## Prompt

Implement a hybrid interactive book component. It should auto-greet the user on page load and then become interactive (clickable) for a full guided sequence. Ensure excellence in both light and dark modes.

## Response snapshot

Engineered a sophisticated hybrid interaction model for the physical textbook's landing page. 1) Created a 3D-tilting SVG book component. 2) Implemented a `useEffect` hook to trigger an "Auto-Greeting" after 2 seconds, where the Agent pops out and waves. 3) Developed a clickable state machine that cycles through a "Guide Sequence" (Hello -> Ready? -> Let's Go!). 4) Styled with theme-aware CSS to ensure speech bubbles and gradients remain premium in Light and Dark modes.

## Outcome

- ✅ Impact: Dramatically increased "First Impression" hook. The book now feels "alive" and proactive.
- 🧪 Tests: Successful `npm run build`; logical sequencing verified.
- 📁 Files: Updated index.tsx and index.module.css.
- 🔁 Next prompts: Final confirmation of animation smoothness or further content alignment.
- 🧠 Reflection: Proactive interaction (auto-greeting) is a high-level UI pattern that communicates "Intelligence" better than static content ever could.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Add spatial audio cues for the popping agent.
