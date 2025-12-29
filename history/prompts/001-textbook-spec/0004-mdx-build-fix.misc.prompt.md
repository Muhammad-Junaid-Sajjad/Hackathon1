---
id: 0004
title: MDX Build Fix and Constitution Update
stage: misc
date: 2025-12-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-textbook-spec
user: junaid
command: continue where left off - fix MDX build errors
labels: ["build","docusaurus","mdx","constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.ts
  - package.json
  - docs/M1/C3/S3.md
  - .specify/memory/constitution.md
tests:
  - npm run build (SUCCESS)
---

## Prompt

"ok plese continue where ou left off step by step ok" and later "wel whet do you think we should do next es per our constitution" and "ok proceed with option 2" (keep Docusaurus, fix MDX issues)

## Response snapshot

Fixed persistent `ReferenceError: i is not defined` MDX build error by:
1. Identified root cause: `{i}` pattern in DH parameters table being interpreted as JSX
2. Fixed by changing `{i}` to `&#95;i` (HTML entity for underscore) in table cells
3. Updated constitution Article IX with MDX handling rules:
   - Curly braces `{variable}` in prose/tables → `&#95;variable` or remove braces
   - Comparison operators in prose text → `&lt;`, `&gt;`
4. Build now passes with all 84 sections
5. Verified HTML output shows correct Docusaurus structure with sidebar navigation

## Outcome

- ✅ Impact: Build now successful, all 84 sections deployable
- 🧪 Tests: `npm run build` passes without errors
- 📁 Files: Modified 4 files (constitution, docusaurus.config.ts, package.json, docs/M1/C3/S3.md)
- 🔁 Next prompts: Deploy to GitHub Pages
- 🧠 Reflection: Docusaurus MDX interprets `{variable}` as JSX expression. Must escape curly braces in prose text/tables with HTML entities.

## Evaluation notes (flywheel)

- Failure modes observed: MDX parsing `{}` as JSX expression, causing ReferenceError during SSG
- Graders run and results (PASS/FAIL): `npm run build` PASS
- Prompt variant (if applicable): N/A
- Next experiment (if needed): Test deployment to GitHub Pages
