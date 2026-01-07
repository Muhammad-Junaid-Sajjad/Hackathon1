# Claude Code Rules

This file is generated during init for the selected agent.

You are an expert AI assistant specializing in Spec-Driven Development (SDD). Your primary goal is to work with the architext to build products.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps. 

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `history/state/state.md` — Conversation state management
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.

## Intelligent File Management Guidelines (Matured Decision Framework)

### File Preservation Intelligence:
1. **Content Integrity**: Always verify that book content, documentation, and educational materials are preserved
2. **Dependency Analysis**: Before any file operation, analyze dependencies and import statements
3. **Impact Assessment**: Evaluate the impact of any deletion on project functionality
4. **Recovery Strategy**: Ensure that deleted files can be regenerated or recovered
5. **Verification Protocol**: Confirm project functionality after any file operations

### Context-Aware File Classification:
- **Category A (Immutable)**: Core content files that must never change or be deleted (docs/, book content)
- **Category B (Regenerable)**: Build artifacts and dependencies that can be recreated (node_modules/, dist/, build/)
- **Category C (Verifiable)**: Files requiring usage verification before modification (configuration files, imports)
- **Category D (Disposable)**: Temporary or development-only files that are safe to remove

### Decision-Making Heuristics:
1. **Dependency Chain Analysis**: Check all import/require statements before deletion
2. **Usage Verification**: Confirm files are not referenced elsewhere in the codebase
3. **Functionality Testing**: Verify that project still works after file operations
4. **Content Preservation**: Prioritize preservation of educational content and user data
5. **Reversibility**: Only make changes that can be reversed if needed

### Project-Specific Guidelines (Hackathon1):

#### File Management Policy:
1. **Book Content Preservation**: All files in `docs/` directory are critical educational content and must never be deleted
2. **Website Functionality**: All files in `src/` directory maintain website functionality and user experience
3. **API Dependencies**: Files in `api/` directory support backend services and AI features
4. **Asset Management**: Files in `static/` directory provide essential website assets and images
5. **Configuration Files**: All configuration files (docusaurus.config.ts, package.json, etc.) are required for proper operation

#### File Classification System:
- **Category A (Critical)**: All documentation, source code, and configuration files
- **Category B (Build)**: Dependencies that can be regenerated (node_modules, package-lock.json)
- **Category C (Investigate)**: Files requiring usage verification before modification
- **Category D (Development)**: Optional files for development environment only

#### Project Structure:
- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `history/state/state.md` — Conversation state management
- `.specify/` — SpecKit Plus templates and scripts
- `docs/` — Educational content and documentation
- `src/` — Source code and components
- `api/` — Backend services and APIs
- `static/` — Static assets and images
- `history/` — Project history and records

### Intelligent Analysis Protocol:
1. **Comprehensive Scan**: Always scan the entire project structure before making file decisions
2. **Cross-Reference Check**: Verify that no other files depend on the target file
3. **Risk Assessment**: Evaluate the risk level of each proposed deletion
4. **Stakeholder Impact**: Consider the impact on users and project functionality
5. **Recovery Plan**: Have a recovery plan before executing any deletions

### Revolutionary Digital Textbook Guidelines:

#### Book Title: Physical AI & Humanoid Robotics Book
- **Author**: Muhammad Junaid Sajjad
- **Institution**: Panaversity
- **No Advisor**: Solo authorship throughout the project

#### Digital Textbook Transformation:
1. **From Physical to Digital**: Move from bulky physical books to optimized digital format
2. **AI-Powered Learning**: RAG-based chatbot for instant Q&A from book content
3. **Interactive Experience**: Live VS Code simulation and multimedia content
4. **Space Optimization**: 87% size reduction (572MB → 72MB) while preserving 100% content
5. **Standardized Structure**: Following comprehensive documentation template for consistency

#### Documentation Template Standards:
- **Modular Structure**: Each concept is contained in discrete, reusable modules
- **Interactive Elements**: Embedded code playgrounds, simulations, and quizzes
- **AI Integration**: RAG-enabled Q&A functionality for each section
- **Multimedia Support**: Videos, diagrams, and interactive visualizations
- **Accessibility**: Multi-language support and responsive design
- **Version Control**: Track changes and updates with clear versioning

#### Learning Module Template:
- **Header Section**: Module Title, Learning Objectives, Prerequisites, Completion Time
- **Theory Section**: Core Concepts, Mathematical Foundations, Applications
- **Practice Section**: Tutorials, Code Examples, Simulations, Troubleshooting
- **Assessment Section**: Quizzes, Exercises, Projects, Reviews
- **Resource Section**: References, Reading Materials, Video Content, Links

#### AI Integration Standards:
- **RAG Indexing**: All content indexed for AI-powered search
- **Context Boundaries**: Clear boundaries for AI response generation
- **Source Attribution**: All AI-generated content cites sources
- **Quality Assurance**: Fact-checking and validation protocols

#### Performance Metrics:
- **Engagement Tracking**: Time spent, interaction rates, completion metrics
- **Learning Assessment**: Quiz scores, project evaluations, progress tracking
- **AI Effectiveness**: Query success rates, response accuracy, user satisfaction
- **Content Updates**: Version tracking, improvement metrics, feedback integration

#### Revolutionary Features:
- **RAG-Based Q&A**: Answers questions directly from book content
- **Personalized Learning**: Adaptive content based on individual patterns
- **Real-Time Assistance**: Always-available learning support
- **Space Efficiency**: Optimized delivery with significant size reduction
- **Multimedia Integration**: Videos, simulations, and interactive elements
- **Instant Accessibility**: Always-available learning support without dependencies
