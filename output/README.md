# Output Directory

This directory contains all generated outputs, reports, and runtime artifacts.

**DO NOT commit generated files to git.**

## Structure

```
output/
├── reports/          # Test reports, analysis reports, validation reports
├── metrics/          # Performance metrics, test metrics, resource budgets
├── benchmarks/       # Benchmark results, performance data
└── logs/             # Runtime logs (separate from build logs)
```

## Usage

All test scripts, performance tools, and monitoring systems should write their
outputs to the appropriate subdirectory here.

### Test Reports
```python
output_path = "output/reports/test_report.json"
```

### Performance Metrics
```python
output_path = "output/metrics/performance_metrics.json"
```

### Benchmarks
```python
output_path = "output/benchmarks/benchmark_results.json"
```

### Logs
```python
output_path = "output/logs/application.log"
```

## .gitignore

All generated files in this directory are ignored by git (except this README).
See `.gitignore` for details.
