<!--
Sync Impact Report:
- Version change: N/A → 1.0.0 (initial constitution)
- Added sections: All principles and governance sections
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Technical Accuracy First
All technical content must match official documentation from ROS 2, Gazebo, Isaac, Unity, and OpenAI APIs. No speculative or unverified technical claims are permitted. This ensures readers receive reliable, actionable information that matches current industry standards.

### Educational Clarity
All content must be written at a grade 10-12 reading level with clear textbook-style explanations. Complex concepts should be broken down into digestible sections with practical examples. This ensures accessibility for a broad range of readers.

### Runnable Code Standards
All code examples must be tested and runnable in the specified technology stack (ROS 2 + Python + OpenAI SDK). Code must include proper error handling, documentation, and be ready for direct implementation. This ensures practical value for readers.

### RAG Integrity
The embedded chatbot must strictly answer from book content or user-selected text only, with zero hallucination tolerance. All responses must cite specific section IDs for verification. This maintains trust and accuracy in the AI assistance.

### Multi-Platform Integration
All systems must demonstrate integration across the specified platforms (ROS 2, Gazebo, Unity, Isaac) with clear connection points and compatibility. This ensures comprehensive coverage of the physical AI ecosystem.

### Deployment Reliability
All components must deploy successfully to specified platforms (Book → GitHub Pages; Backend → Vercel/Fly.io) with proper error handling and monitoring. This ensures accessibility and maintainability.

## Technical Standards
All implementations must follow the specified technology stack: Docusaurus for book format, FastAPI for backend, Neon for database, Qdrant for vector storage, and OpenAI Agents for RAG functionality. All diagrams require text descriptions for accessibility compliance.

## Development Workflow
All features must be testable with clear acceptance criteria. Each module must include runnable code examples, integration tests, and deployment validation. Code reviews must verify compliance with all constitution principles before approval.

## Governance
This constitution supersedes all other practices and guidelines for this project. All pull requests and code reviews must verify compliance with these principles. Any changes to this constitution require explicit approval and documentation of the reasoning.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09
