---
id: 1
title: Create Physical AI Constitution
stage: constitution
date: 2025-12-09
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: Farheen00 <farheenusman2@gmail.com>
command: /sp.constitution
labels: [constitution, physical-ai, humanoid-robotics, book]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
/sp.constitution

Project: Physical AI & Humanoid Robotics — Book + Embedded RAG Chatbot

Core principles:
- Technical accuracy (ROS 2, Gazebo, Unity, Isaac, VLA)
- Clear textbook-style explanations
- Consistent structure using Spec-Kit Plus + Claude Code
- RAG answers only from book or user-selected text (no hallucinations)

Key standards:
- Claims must match official docs (ROS 2, Gazebo, Isaac, OpenAI APIs)
- Writing clarity: grade 10–12
- Code must be runnable (ROS 2 + Python + OpenAI SDK)
- RAG chatbot must cite section IDs
- Deployment: Book → GitHub Pages; Backend → Vercel/Fly.io

Constraints:
- Format: Docusaurus
- Length: 150+ pages across 4 modules + Capstone
- Backend stack: FastAPI, Neon, Qdrant, OpenAI Agents
- Chatbot restricted strictly to retrieved content
- All diagrams require text descriptions

Success criteria:
- Book compiles with Spec-Kit Plus
- GitHub Pages deploys without errors
- Chatbot passes: 0% hallucination + ≥95% retrieval precision
- End-to-end humanoid capstone (voice → plan → navigate → detect → manipulate) fully described
```

## Response snapshot

Created a comprehensive constitution file for the Physical AI & Humanoid Robotics project with six core principles:

1. Technical Accuracy First - ensuring all content matches official documentation
2. Educational Clarity - maintaining grade 10-12 reading level
3. Runnable Code Standards - all code examples must be testable
4. RAG Integrity - zero hallucination tolerance for the chatbot
5. Multi-Platform Integration - demonstrating integration across platforms
6. Deployment Reliability - ensuring successful deployment to specified platforms

Also included technical standards, development workflow, and governance sections.

## Outcome

- ✅ Impact: Created initial project constitution with 6 core principles and governance structure
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with project-specific principles
- 🔁 Next prompts: Plan creation, specification development
- 🧠 Reflection: Constitution provides clear guidelines for maintaining technical accuracy and educational quality

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on constitution