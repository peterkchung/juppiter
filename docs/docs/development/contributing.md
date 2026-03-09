# Contributing Guidelines

Thank you for contributing to juppiter! This guide covers our workflow, coding standards, and review process.

## Branch Strategy

```
main (stable releases only)
  ↑ PR with squash merge
  ↑
develop (integration branch)
  ↑ PR with regular merge
  ↑
feature/my-feature
```

### Branch Rules

- **main**: Stable releases only. PRs from `develop` only.
- **develop**: Integration branch. Feature PRs merge here.
- **feature/***: Branch from `develop`, PR back to `develop`.

### Creating a Feature Branch

```bash
# Ensure you're on develop and up to date
git checkout develop
git pull origin develop

# Create and switch to feature branch
git checkout -b feature/my-feature-name

# Work on your changes...

# Push to your fork
git push -u origin feature/my-feature-name
```

## Workflow

### 1. Before Starting

- Check [existing issues](https://github.com/opencode/juppiter/issues) or create one
- Comment on issue to claim it
- Discuss approach in issue if complex

### 2. Development

```bash
# Enter container
docker compose -f docker/docker-compose.yml exec dev bash

# Build
colcon build --symlink-install

# Make changes...

# Test
ros2 launch gazebo_lunar_sim simulation.launch.py

# Run unit tests
colcon test --packages-select <your_package>
colcon test-result --verbose
```

### 3. Commit

**Commit Message Format:**
```
type(scope): subject

body (optional)

footer (optional)
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes (formatting, semicolons)
- `refactor`: Code refactoring
- `test`: Adding or fixing tests
- `chore`: Maintenance tasks

**Examples:**
```
feat(sensor_core): add async health monitoring

Implement non-blocking health checks using timer callbacks.
Reduces main thread latency by ~2ms.

fix(fusion_core): correct weight normalization

Weights were not properly normalized when health dropped below
threshold, causing sum != 1.0.

docs(readme): update Docker instructions

Add section on Windows WSL2 setup with WSLg for GUI forwarding.
```

### 4. Push and PR

```bash
# Push to your fork
git push origin feature/my-feature-name

# Create PR via GitHub
# Target branch: develop
```

**PR Template:**
```markdown
## Description
Brief description of changes

## Related Issue
Closes #123

## Testing
- [ ] Unit tests pass
- [ ] Integration tests pass
- [ ] Tested in simulation
- [ ] Manual testing completed

## Checklist
- [ ] Code follows style guidelines
- [ ] Documentation updated
- [ ] Tests added/updated
- [ ] No new compiler warnings
- [ ] CHANGELOG.md updated (if applicable)
```

### 5. Review Process

**Reviewer Checklist:**
- [ ] Code quality and style
- [ ] Tests cover changes
- [ ] Documentation updated
- [ ] No regressions
- [ ] Performance impact acceptable

**Auto-merge:**
- Enabled for `develop` branch after approval
- Requires passing CI checks
- Requires 1 approving review

## Code Style

### C++

Follow [ROS 2 C++ Style Guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html):

**Formatting:**
```cpp
// Indentation: 2 spaces
// Braces: Allman style
class MyClass {
 public:
  void myMethod();
  
 private:
  int member_variable_;  // trailing underscore for members
};

// Function names: camelCase
void myFunction() {
  if (condition) {
    doSomething();
  }
}
```

**Naming:**
- Classes: `PascalCase`
- Functions: `camelCase`
- Variables: `snake_case`
- Member variables: `snake_case_`
- Constants: `kPascalCase`
- Macros: `UPPER_CASE`

**Headers:**
```cpp
// Include guards
#ifndef PACKAGE_NAME_FILE_NAME_HPP_
#define PACKAGE_NAME_FILE_NAME_HPP_

// Standard headers first
#include <vector>
#include <string>

// ROS headers
#include <rclcpp/rclcpp.hpp>

// Local headers
#include "my_package/my_header.hpp"

namespace my_package {

class MyClass {
  // ...
};

}  // namespace my_package

#endif  // PACKAGE_NAME_FILE_NAME_HPP_
```

### YAML

```yaml
# 2-space indentation
compute_profile:
  name: "dev"
  hardware_target: "workstation"
  
  # Use spaces, not tabs
  estimators:
    lio:
      implementation: "fast_lio2"
      enabled: true
```

### Python

Follow PEP 8:
```python
# 4-space indentation
def my_function(param1, param2):
    """Docstring description."""
    if condition:
        do_something()
    return result
```

## Documentation

### Required Documentation

**For new features:**
- Update relevant architecture docs
- Add configuration examples
- Update CHANGELOG.md

**For bug fixes:**
- Explain fix in commit message
- Update docs if behavior changes

### Documentation Structure

```
docs/
├── architecture/    # System design
├── user-guides/     # How-to guides
├── development/     # Contributing
└── reference/       # API docs
```

## Testing Requirements

### Minimum Test Coverage

| Component | Required |
|-----------|----------|
| Core libraries | 80% coverage |
| Sensor drivers | Unit tests |
| Estimators | Integration tests |
| FDIIR logic | Scenario tests |

### Running Tests

```bash
# All tests
colcon test

# Specific package
colcon test --packages-select sensor_core

# With verbose output
colcon test --packages-select sensor_core --event-handlers console_direct+

# View results
colcon test-result --verbose
```

### Test Categories

**Unit Tests:** Fast, isolated, no ROS dependencies
```cpp
TEST(MyClassTest, Initialization) {
    MyClass obj;
    EXPECT_TRUE(obj.initialize());
}
```

**Integration Tests:** Test component interactions
```bash
ros2 test my_package test_integration.launch.py
```

**FDIIR Tests:** Validate fault handling (see [FDIIR Testing](../user-guides/fdiir-testing.md))

## Release Process

### Version Numbering

Semantic versioning: `MAJOR.MINOR.PATCH`
- **MAJOR**: Breaking API changes
- **MINOR**: New features, backwards compatible
- **PATCH**: Bug fixes, backwards compatible

### Creating a Release

```bash
# 1. Update version in package.xml files
# 2. Update CHANGELOG.md
# 3. Create PR to main
# 4. After merge, tag release
git tag -a v1.0.0 -m "Release version 1.0.0"
git push origin v1.0.0
```

## Questions?

- Open an [issue](https://github.com/opencode/juppiter/issues)
- Check [existing documentation](../architecture/index.md)
- Review [troubleshooting guide](troubleshooting.md)

## License

By contributing, you agree that your contributions will be licensed under the Apache 2.0 License.
