# URC 2026 Testing Pyramid

## 🏗️ Testing Hierarchy Overview

This project follows a **three-layer testing pyramid** - the industry standard for comprehensive software validation:

```
🎯 Unit Tests (Layer 1)
    ↓ Fast feedback, individual components
🔗 Integration Tests (Layer 2)
    ↓ Component interactions, data flow
🚀 Simulation Tests (Layer 3)
    ↓ Real-world conditions, system validation
```

## 🚀 Quick Commands

### Run Complete Testing Pyramid
```bash
python3 test_everything.py
```

### Run Individual Test Suites
```bash
python3 tests/run_tests.py --unit        # Unit tests (~5 minutes)
python3 tests/run_tests.py --integration # Integration tests (~20 minutes)
python3 tests/run_tests.py --system      # System tests (~15 minutes)
python3 tests/run_tests.py --performance # Performance tests (~10 minutes)
```

## 📊 What Each Layer Tests

| Layer | Purpose | Duration | Files | Tests |
|-------|---------|----------|-------|-------|
| **Unit** | Individual components with mocks | 2-5 min | 11 | ~400 |
| **Integration** | Component interactions | 10-30 min | 28 | ~200 |
| **Simulation** | System under real conditions | 15-60 min | 1 | 18 |

## 🎯 When to Run Each Layer

- **Unit Tests**: After every code change (fast feedback)
- **Integration Tests**: Before commits, after feature completion
- **Simulation Tests**: Before hardware access, major releases
- **Full Pyramid**: Comprehensive validation, CI/CD pipelines

## 📋 Test Results

Results are saved to `test_reports/` with detailed JSON reports for each layer.

## ⚠️ Important Notes

- **Pyramid stops on failure**: Fix lower layers before proceeding up
- **Simulation tests are NOT hardware validation**: Real hardware testing still required
- **All tests run without hardware**: Complete software validation before physical components

## 🆘 Troubleshooting

- Unit test failures: Check individual component logic
- Integration failures: Check component interfaces and data flow
- Simulation failures: Check system behavior under stress/edge conditions

---

**Ready to validate your URC rover software? Start with `./tests/run_all_tests.py`!**
