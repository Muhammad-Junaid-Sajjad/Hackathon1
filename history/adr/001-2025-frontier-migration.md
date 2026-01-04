# ADR-001: Migration to 2025 Frontier Robotics Stack

- **Status**: Accepted
- **Date**: 2026-01-04
- **Deciders**: AI Assistant, Human Architect

## Context and Problem Statement

The project was originally conceived using a 2024 technical baseline (Ubuntu 22.04, ROS 2 Humble, NVIDIA RTX 40-series). During the implementation phase, the project team decided to leapfrog into the 2025 frontier stack to ensure the textbook remains relevant for the next 24 months.

## Decision Drivers

- Availability of **ROS 2 Kilted Kaiju** (2025) which offers superior humanoid-specific middleware support.
- The release of **Ubuntu 24.04 LTS** as the new industry-standard baseline for AI robotics.
- The requirement for **Blackwell Architecture** (RTX 50-series) to support next-generation VLA (Vision-Language-Action) foundation models.
- The introduction of the **NVIDIA Thor** platform for humanoid SoCs.

## Considered Options

1. **Maintain 2024 Stack**: Stable, well-documented, but rapidly becoming obsolete.
2. **Hybrid Stack**: Mix of old OS with new middleware (risky dependency hell).
3. **Full 2025 Migration (Chosen)**: Leapfrog to the latest standards for maximum future-proofing.

## Decision Outcome

**Full 2025 Migration**. We have updated the project Constitution and Technical Specifications to reflect this new "Law."

### Consequences

- **Pros**: Pedagogical leadership; alignment with 2025/2026 industry standards; support for larger AI models.
- **Cons**: Increased workstation requirements (RTX 50xx); higher edge kit costs (~$950 vs $700).
- **Risks**: Bleeding-edge driver stability for Blackwell/Thor.

## Implementation Notes

- Middleware initialized with ROS 2 Kilted Kaiju.
- Simulation target shifted to Gazebo Ionic (9.x).
- Documentation hierarchy expanded from 84 to 87 sections to cover Frontier Research.
