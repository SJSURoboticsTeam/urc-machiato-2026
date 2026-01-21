#!/bin/bash
# Quality Check Script - Run before committing code
# This script enforces the quality standards defined in docs/quality_standards.rst

set -e  # Exit on any error

echo "🔍 URC 2026 Quality Check Script"
echo "================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print status
print_status() {
    local status=$1
    local message=$2
    case $status in
        "PASS")
            echo -e "${GREEN}✅ PASS${NC}: $message"
            ;;
        "FAIL")
            echo -e "${RED}❌ FAIL${NC}: $message"
            ;;
        "WARN")
            echo -e "${YELLOW}⚠️  WARN${NC}: $message"
            ;;
        "INFO")
            echo -e "${BLUE}ℹ️  INFO${NC}: $message"
            ;;
    esac
}

# Check if required tools are installed
check_dependencies() {
    echo "📦 Checking dependencies..."

    local missing_tools=()

    if ! command -v black &> /dev/null; then
        missing_tools+=("black")
    fi

    if ! command -v isort &> /dev/null; then
        missing_tools+=("isort")
    fi

    if ! command -v flake8 &> /dev/null; then
        missing_tools+=("flake8")
    fi

    if ! command -v mypy &> /dev/null; then
        missing_tools+=("mypy")
    fi

    if ! command -v pytest &> /dev/null; then
        missing_tools+=("pytest")
    fi

    if [ ${#missing_tools[@]} -ne 0 ]; then
        print_status "FAIL" "Missing required tools: ${missing_tools[*]}"
        print_status "INFO" "Install with: pip install black isort flake8 mypy pytest pytest-cov"
        return 1
    fi

    print_status "PASS" "All required tools are installed"
    return 0
}

# Code formatting check
check_formatting() {
    echo ""
    echo "🎨 Checking code formatting..."

    if black --check --quiet .; then
        print_status "PASS" "Code formatting (black)"
    else
        print_status "FAIL" "Code formatting issues found"
        print_status "INFO" "Fix with: black ."
        return 1
    fi

    return 0
}

# Import sorting check
check_imports() {
    echo ""
    echo "📚 Checking import sorting..."

    if isort --check-only --quiet .; then
        print_status "PASS" "Import sorting (isort)"
    else
        print_status "FAIL" "Import sorting issues found"
        print_status "INFO" "Fix with: isort ."
        return 1
    fi

    return 0
}

# Linting check
check_linting() {
    echo ""
    echo "🔍 Checking code linting..."

    # Create a temporary file to capture output
    local lint_output
    lint_output=$(flake8 . --max-line-length=88 --extend-ignore=E203,W503 2>&1) || true

    if [ -z "$lint_output" ]; then
        print_status "PASS" "Code linting (flake8)"
    else
        print_status "FAIL" "Linting issues found"
        echo "$lint_output" | head -20
        if [ $(echo "$lint_output" | wc -l) -gt 20 ]; then
            echo "... (truncated - see full output above)"
        fi
        return 1
    fi

    return 0
}

# Type checking
check_types() {
    echo ""
    echo "🏷️  Checking type hints..."

    # Run mypy with less strict settings for gradual adoption
    if mypy . --ignore-missing-imports --no-strict-optional --quiet; then
        print_status "PASS" "Type checking (mypy)"
    else
        print_status "WARN" "Type checking found issues (may be acceptable)"
        print_status "INFO" "Review mypy output above - some issues may be expected"
        # Don't fail on type checking for gradual adoption
    fi

    return 0
}

# Unit tests
run_unit_tests() {
    echo ""
    echo "🧪 Running unit tests..."

    if [ -d "tests/unit" ]; then
        if pytest tests/unit/ -v --tb=short --quiet; then
            print_status "PASS" "Unit tests"
        else
            print_status "FAIL" "Unit tests failed"
            return 1
        fi
    else
        print_status "WARN" "No unit tests directory found"
    fi

    return 0
}

# Integration tests (optional, faster check)
run_integration_tests() {
    echo ""
    echo "🔗 Running integration tests..."

    if [ -d "tests/integration" ]; then
        if pytest tests/integration/ -v --tb=short --quiet --maxfail=3; then
            print_status "PASS" "Integration tests"
        else
            print_status "WARN" "Integration tests failed (may be expected)"
            # Don't fail on integration tests as they may require full setup
        fi
    else
        print_status "INFO" "No integration tests directory found"
    fi

    return 0
}

# ROS2 package check
check_ros2_packages() {
    echo ""
    echo "🤖 Checking ROS2 packages..."

    if command -v colcon &> /dev/null; then
        # Quick syntax check of package.xml files
        if find src -name "package.xml" -exec xmllint --noout {} \; 2>/dev/null; then
            print_status "PASS" "ROS2 package.xml syntax"
        else
            print_status "WARN" "ROS2 package.xml syntax issues"
        fi
    else
        print_status "INFO" "colcon not available - skipping ROS2 checks"
    fi

    return 0
}

# Documentation check
check_docs() {
    echo ""
    echo "📚 Checking documentation..."

    if [ -d "docs" ]; then
        if command -v sphinx-build &> /dev/null; then
            if (cd docs && make html >/dev/null 2>&1); then
                print_status "PASS" "Documentation build"
            else
                print_status "WARN" "Documentation build issues"
            fi
        else
            print_status "INFO" "Sphinx not available - skipping docs check"
        fi
    else
        print_status "INFO" "No docs directory found"
    fi

    return 0
}

# Coverage check
check_coverage() {
    echo ""
    echo "📊 Checking test coverage..."

    if [ -d "tests" ]; then
        if pytest --cov=. --cov-report=term-missing --cov-fail-under=80 --quiet tests/ 2>/dev/null; then
            print_status "PASS" "Test coverage (>80%)"
        else
            print_status "WARN" "Test coverage below 80%"
            # Don't fail on coverage for gradual improvement
        fi
    else
        print_status "WARN" "No tests directory found"
    fi

    return 0
}

# Main execution
main() {
    local start_time=$(date +%s)
    local failures=0

    echo "🚀 Starting quality checks..."
    echo ""

    # Check dependencies first
    if ! check_dependencies; then
        exit 1
    fi

    # Run checks (continue even if some fail to show all issues)
    check_formatting || ((failures++))
    check_imports || ((failures++))
    check_linting || ((failures++))
    check_types || true  # Don't count type checking failures
    run_unit_tests || ((failures++))
    run_integration_tests || true  # Don't count integration test failures
    check_ros2_packages || true
    check_docs || true
    check_coverage || true

    # Summary
    echo ""
    echo "📋 Quality Check Summary"
    echo "========================"

    local end_time=$(date +%s)
    local duration=$((end_time - start_time))

    if [ $failures -eq 0 ]; then
        print_status "PASS" "All quality checks passed! 🎉"
        echo ""
        echo "✅ Your code is ready for review and commit."
        echo "💡 Next steps:"
        echo "   - Create a pull request"
        echo "   - Request code review"
        echo "   - Follow the process in docs/code_review_process.rst"
    else
        print_status "FAIL" "$failures quality check(s) failed"
        echo ""
        echo "❌ Please fix the issues above before committing."
        echo "💡 Helpful commands:"
        echo "   - Fix formatting: black ."
        echo "   - Fix imports: isort ."
        echo "   - Run tests: pytest tests/unit/"
        echo "   - See docs: docs/quality_standards.rst"
        exit 1
    fi

    echo ""
    echo "⏱️  Total time: ${duration}s"
}

# Run main function
main "$@"




