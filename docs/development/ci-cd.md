# CI/CD Pipeline

juppiter uses GitHub Actions for continuous integration and deployment.

## Overview

```
┌─────────────────────────────────────────────────────┐
│  GitHub Actions Workflow                            │
│                                                     │
│  ┌─────────────┐                                    │
│  │   Push/PR   │                                    │
│  └──────┬──────┘                                    │
│         │                                           │
│         ▼                                           │
│  ┌─────────────┐                                    │
│  │    Build    │  → colcon build                   │
│  └──────┬──────┘                                    │
│         │                                           │
│         ▼                                           │
│  ┌─────────────┐                                    │
│  │    Test     │  → Unit + Integration tests        │
│  └──────┬──────┘                                    │
│         │                                           │
│         ▼                                           │
│  ┌─────────────┐                                    │
│  │    FDIIR    │  → Fault scenario tests          │
│  └──────┬──────┘                                    │
│         │                                           │
│         ▼                                           │
│  ┌─────────────┐                                    │
│  │  Auto-merge │  → If all pass on develop PR      │
│  │  (optional) │                                    │
│  └─────────────┘                                    │
└─────────────────────────────────────────────────────┘
```

## Workflow Files

Located in `.github/workflows/`:

```
.github/workflows/
├── ci.yml              # Main CI pipeline
├── docs.yml            # Documentation build/deploy
└── auto-merge.yml      # Automated merging (optional)
```

## Main CI Pipeline (ci.yml)

```yaml
name: CI

on:
  push:
    branches: [main, develop]
  pull_request:
    branches: [main, develop]

jobs:
  build-and-test:
    runs-on: ubuntu-latest
    container:
      image: ros:kilted

    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-colcon-common-extensions
          rosdep update
          rosdep install --from-paths src --ignore-src -y

      - name: Build
        run: |
          . /opt/ros/kilted/setup.sh
          colcon build --event-handlers console_direct+

      - name: Test
        run: |
          . /opt/ros/kilted/setup.sh
          colcon test --event-handlers console_direct+
          colcon test-result --verbose

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: test-results
          path: build/**/test_results/
```

### What It Does

1. **Checkout** - Pulls code from repository
2. **Install dependencies** - Uses rosdep to install system dependencies
3. **Build** - Compiles all packages with colcon
4. **Test** - Runs all unit and integration tests
5. **Upload results** - Saves test reports as artifacts

## Documentation Pipeline (docs.yml)

```yaml
name: Documentation

on:
  push:
    branches: [main, develop]
    paths:
      - 'docs/**'
      - 'mkdocs.yml'
  pull_request:
    branches: [main, develop]
    paths:
      - 'docs/**'
      - 'mkdocs.yml'

jobs:
  build:
    runs-on: ubuntu-latest
    
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r docs/requirements.txt

      - name: Build documentation
        run: |
          cd docs
          mkdocs build --strict

      - name: Check links
        run: |
          cd docs
          mkdocs build --strict 2>&1 | grep -i "broken" || true

  deploy:
    needs: build
    runs-on: ubuntu-latest
    if: github.ref == 'refs/heads/main'
    
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install -r docs/requirements.txt

      - name: Deploy to GitHub Pages
        run: |
          cd docs
          mkdocs gh-deploy --force
```

### What It Does

1. **Trigger** - Runs when docs/ files change
2. **Build** - Compiles MkDocs site with strict mode
3. **Link check** - Validates all internal links
4. **Deploy** - Pushes to `gh-pages` branch (only on main)

## Auto-Merge (auto-merge.yml)

Optional workflow for automatic merging:

```yaml
name: Auto-merge

on:
  pull_request:
    types:
      - labeled
      - unlabeled
      - synchronize
      - opened
      - edited
      - ready_for_review
      - reopened
      - unlocked
  pull_request_review:
    types:
      - submitted
  check_suite:
    types:
      - completed
  status: {}

jobs:
  auto-merge:
    runs-on: ubuntu-latest
    if: github.event.pull_request.base.ref == 'develop'
    
    steps:
      - name: Auto-merge PR
        uses: pascalgn/automerge-action@v0.15.6
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
          MERGE_LABELS: "automerge"
          MERGE_METHOD: "merge"
          MERGE_COMMIT_MESSAGE: "pull-request-title"
          MERGE_FORKS: "false"
          MERGE_RETRIES: "6"
          MERGE_RETRY_SLEEP: "10000"
```

### How to Use

1. Create PR to `develop`
2. Add label `automerge`
3. Get approval from reviewer
4. CI passes → auto-merges

## Status Checks

Required checks for PRs:

- [ ] **Build** - Must compile without errors
- [ ] **Unit Tests** - All tests pass
- [ ] **Integration Tests** - Component tests pass
- [ ] **Documentation** - MkDocs builds successfully

## Local CI Simulation

Test CI locally before pushing:

```bash
# Install act (GitHub Actions local runner)
curl https://raw.githubusercontent.com/nektos/act/master/install.sh | sudo bash

# Run CI workflow locally
act -j build-and-test

# Run specific workflow
act -W .github/workflows/ci.yml
```

## Troubleshooting CI Failures

### Build Failures

```bash
# Common issues:
# 1. Missing dependencies in package.xml
# 2. Syntax errors in code
# 3. Missing includes

# Fix and retry
git commit -am "fix: resolve build error"
git push
```

### Test Failures

```bash
# Download artifact from failed run
# (GitHub Actions → Build → Artifacts)

# Run locally to debug
colcon test --packages-select <failed_package>
colcon test-result --verbose
```

### Documentation Failures

```bash
# Test locally
cd docs
mkdocs build --strict

# Common issues:
# - Broken internal links
# - Missing nav entries in mkdocs.yml
# - Syntax errors in markdown
```

## Pipeline Status

Check status at:
- GitHub repository → Actions tab
- Badges in README.md:
  ```markdown
  ![CI](https://github.com/opencode/juppiter/workflows/CI/badge.svg)
  ![Docs](https://github.com/opencode/juppiter/workflows/Documentation/badge.svg)
  ```

## Release Automation

Future enhancement: Automated releases

```yaml
# .github/workflows/release.yml (example)
name: Release

on:
  push:
    tags:
      - 'v*'

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
      - name: Create Release
        uses: actions/create-release@v1
        env:
          GITHUB_TOKEN: ${{ secrets.GITHUB_TOKEN }}
        with:
          tag_name: ${{ github.ref }}
          release_name: Release ${{ github.ref }}
          draft: false
          prerelease: false
```

## Security

### Secrets

Required secrets (if using deployment):
- `GITHUB_TOKEN` - Automatically provided
- `SSH_PRIVATE_KEY` - For external deployment (if needed)

### Permissions

Workflow permissions are minimal:
- Read: Code, issues, PRs
- Write: PRs (for auto-merge), deployments (for docs)

## Customization

### Adding New Checks

Edit `.github/workflows/ci.yml`:

```yaml
      - name: New check
        run: |
          # Your check here
          python3 scripts/validate_something.py
```

### Matrix Builds

Test on multiple platforms:

```yaml
strategy:
  matrix:
    ros_distro: [kilted, rolling]
    
jobs:
  build:
    runs-on: ubuntu-latest
    container:
      image: ros:${{ matrix.ros_distro }}
```

## Next Topics

- [Contributing](contributing.md) - Code review process
- [Testing](testing.md) - Test structure and writing tests
- [Troubleshooting](troubleshooting.md) - Debugging CI issues
