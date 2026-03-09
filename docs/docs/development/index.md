# Development

Resources for developers contributing to juppiter.

## Topics

- [Contributing](contributing.md) - Branch workflow, code style, PR process
- [Testing](testing.md) - Unit tests, integration tests, test coverage
- [CI/CD](ci-cd.md) - GitHub Actions, automated testing
- [Troubleshooting](troubleshooting.md) - Common development issues

## Quick Links

**Code Style:**
- C++: Follow ROS 2 style guidelines
- YAML: 2-space indentation
- Python (scripts): PEP 8

**Branch Strategy:**
```
main (stable releases)
  ↑
develop (integration)
  ↑
feature/my-feature
```

**Build Commands:**
```bash
# Full build
colcon build --symlink-install

# Single package
colcon build --packages-select sensor_core

# Run tests
colcon test --packages-select sensor_core fusion_core

# View results
colcon test-result --verbose
```

## Getting Started

1. [Fork the repository](https://github.com/opencode/juppiter/fork)
2. Clone your fork
3. Create a feature branch from `develop`
4. Make changes following our guidelines
5. Run tests locally
6. Submit PR to `develop`

See [Contributing](contributing.md) for detailed workflow.
