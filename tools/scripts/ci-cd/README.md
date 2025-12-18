# CI/CD Scripts

Scripts for continuous integration, deployment, and documentation building.

## Scripts

### Docker & Deployment
- **`build_universal_docker.sh`** - Build universal Docker images for different environments

### Documentation
- **`build_docs.sh`** - Build Sphinx documentation from source

## Usage

### Build Docker Images
```bash
./scripts/ci-cd/build_universal_docker.sh
```

### Build Documentation
```bash
./scripts/ci-cd/build_docs.sh
```

## Dependencies

- Docker (for container builds)
- Sphinx and documentation dependencies (for docs build)

## Notes

- Docker builds support multiple architectures and environments
- Documentation builds require docs/ directory and Sphinx configuration
