# Physical AI & Humanoid Robotics Textbook

> Comprehensive 13-week textbook for industry practitioners: ROS 2, Digital Twin (Gazebo/Unity), NVIDIA Isaac Sim, and Vision-Language-Action models.

## Overview

This textbook provides hands-on training for building autonomous humanoid robots using:
- **ROS 2** (Weeks 3-5): Robot Operating System fundamentals
- **Digital Twin** (Weeks 6-7): Gazebo and Unity simulation
- **NVIDIA Isaac Sim** (Weeks 8-10): GPU-accelerated simulation and synthetic data
- **Vision-Language-Action Models** (Weeks 11-13): Multimodal AI for humanoid control

**Target Audience**: Industry practitioners with Python programming knowledge, transitioning to robotics and embodied AI.

## Course Structure

| Module | Weeks | Focus |
|--------|-------|-------|
| Introduction | 1-2 | Physical AI Foundations |
| Module 1: ROS 2 | 3-5 | Architecture, Topics, URDF |
| Module 2: Digital Twin | 6-7 | Gazebo, Unity, Sim2Real |
| Module 3: NVIDIA Isaac | 8-10 | Isaac Sim, Synthetic Data, Imitation Learning |
| Module 4: VLA & Humanoids | 11-13 | Multimodal Models, Transformer Policies |
| Capstone | Week 13 | Autonomous Humanoid (Voice → Action) |

## Hardware Paths

Choose one of three hardware configurations:

1. **Digital Twin Workstation**: RTX 3060+ GPU, Ubuntu 22.04
2. **Physical AI Edge Kit**: NVIDIA Jetson Orin Nano
3. **Cloud-Native**: AWS/Azure with GPU instances

## Documentation Site

This project uses [Docusaurus 3](https://docusaurus.io/) with:
- Dashboard-style homepage with module cards
- Nested sidebar with collapsible categories
- Hybrid search (Algolia + Flexsearch for glossary)
- Custom metadata for chapter prerequisites and learning objectives
- GitHub Actions CI/CD with quality gates

## Project Structure

```
Physical-AI-Humanoid-Robotics-Textbook/
├── docs/                   # Main content
│   ├── intro.md
│   ├── setup/              # 3 hardware paths
│   ├── module-1-ros2/      # Weeks 3-5
│   ├── module-2-digital-twin/   # Weeks 6-7
│   ├── module-3-isaac/     # Weeks 8-10
│   ├── module-4-vla-humanoids/  # Weeks 11-13
│   ├── capstone/
│   └── references/         # Glossary, notation, troubleshooting
├── src/                    # Custom React components
│   ├── components/
│   └── pages/index.tsx     # Dashboard homepage
├── specs/                  # Feature specifications
│   └── 001-book-master-plan/
│       ├── spec.md
│       ├── plan.md
│       ├── tasks.md
│       └── contracts/      # JSON Schema for validation
└── .github/workflows/      # CI/CD pipelines
```

## Contributing

We welcome contributions! Please see the [Quickstart Guide](specs/001-book-master-plan/quickstart.md) for:
- Development setup
- Creating new chapters
- Running quality checks
- Metadata validation

### Quality Gates

All PRs must pass:
- Build with 0 errors/warnings
- Link validation (0 broken links)
- Lighthouse scores: Performance ≥90, Accessibility ≥95, SEO ≥95
- Chapter metadata validation against JSON Schema

## Specification-Driven Development

This project follows **Spec-Driven Development (SDD)**
1. **Constitution**: Core principles in [`.specify/memory/constitution.md`](.specify/memory/constitution.md)
2. **Specifications**: Feature specs in [`specs/001-book-master-plan/spec.md`](specs/001-book-master-plan/spec.md)
3. **Planning**: Implementation plan in [`specs/001-book-master-plan/plan.md`](specs/001-book-master-plan/plan.md)
4. **Tasks**: Breakdown in [`specs/001-book-master-plan/tasks.md`](specs/001-book-master-plan/tasks.md)
5. **History**: Prompt History Records in [`history/prompts/`](history/prompts/)

## Technology Stack

- **Documentation**: Docusaurus 3.x
- **Language**: TypeScript 5.x
- **UI**: React 18.x
- **Build Tools**: Node.js 18+
- **Search**: Algolia DocSearch + Flexsearch
- **CI/CD**: GitHub Actions
- **Deployment**: GitHub Pages
- **AI Assistant**: Claude Code

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


