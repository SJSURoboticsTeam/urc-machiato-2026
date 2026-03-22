# Production Scripts

Scripts for production deployment, validation, and system health checks.

## Scripts

### Core Validation

- **`validate_config.py`** - Validates configuration files for production readiness and environment consistency
- **`production_health_check.py`** - Comprehensive system health check for deployment validation

### GitHub Integration

- **`extract-todos-to-issues.py`** - Automated TODO extraction from `*_TODO.md` files and GitHub issue creation

## Usage

### Configuration Validation

```bash
python3 scripts/production/validate_config.py
```

### Production Health Check

```bash
python3 scripts/production/production_health_check.py
```

### TODO Extraction

```bash
python3 scripts/production/extract-todos-to-issues.py
```

## Dependencies

- `validate_config.py`: PyYAML
- `production_health_check.py`: None
- `extract-todos-to-issues.py`: requests, GitHub token in environment

## Notes

- These scripts are designed for production deployment workflows
- Some require specific environment variables or API tokens
- Scripts validate system readiness before deployment
