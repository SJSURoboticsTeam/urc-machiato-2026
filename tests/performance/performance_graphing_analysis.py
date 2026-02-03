#!/usr/bin/env python3
"""
Performance Graphing and Analysis - URC 2026 Mars Rover

Creates comprehensive visualizations and analysis from performance test data,
including chaos engineering results and statistical analysis.

Usage:
    python performance_graphing_analysis.py --input-report performance_baseline_report_*.json --output-dir graphs/

Author: URC 2026 Performance Engineering Team
"""

import argparse
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
from pathlib import Path
from typing import Dict, List, Any, Optional
import statistics
from datetime import datetime


class PerformanceGraphGenerator:
    """Generate comprehensive performance graphs and analysis."""

    def __init__(self, output_dir: str = "performance_graphs"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Set up matplotlib style
        plt.style.use("seaborn-v0_8")
        sns.set_palette("husl")

        self.data = {}

    def load_performance_data(self, report_files: List[str]):
        """Load performance data from report files."""
        print("📊 Loading performance data...")

        for report_file in report_files:
            try:
                with open(report_file, "r") as f:
                    report = json.load(f)

                # Extract test results
                if "results" in report:
                    for test_name, test_data in report["results"].items():
                        if test_name not in self.data:
                            self.data[test_name] = []
                        self.data[test_name].append(test_data)

                print(f"✅ Loaded data from {report_file}")

            except Exception as e:
                print(f"⚠️ Failed to load {report_file}: {e}")

        print(f"📈 Loaded data for {len(self.data)} test categories")

    def generate_comprehensive_dashboard(self):
        """Generate comprehensive performance dashboard."""
        print("📊 Generating Comprehensive Performance Dashboard...")

        if not self.data:
            print("❌ No performance data available")
            return

        # Create main dashboard
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(
            "URC 2026 Mars Rover - Performance Analysis Dashboard",
            fontsize=16,
            fontweight="bold",
        )

        # 1. Performance Metrics Overview
        self._plot_performance_overview(axes[0, 0])

        # 2. Latency Distribution
        self._plot_latency_distribution(axes[0, 1])

        # 3. Stress Test Results
        self._plot_stress_test_results(axes[0, 2])

        # 4. Requirements Compliance
        self._plot_requirements_compliance(axes[1, 0])

        # 5. Performance Trends
        self._plot_performance_trends(axes[1, 1])

        # 6. Statistical Summary
        self._plot_statistical_summary(axes[1, 2])

        plt.tight_layout()

        dashboard_file = (
            self.output_dir
            / f"performance_dashboard_{int(datetime.now().timestamp())}.png"
        )
        plt.savefig(dashboard_file, dpi=300, bbox_inches="tight")
        print(f"📊 Dashboard saved to: {dashboard_file}")
        plt.close()

    def _plot_performance_overview(self, ax):
        """Plot performance metrics overview."""
        categories = []
        p99_values = []
        mean_values = []

        for test_name, test_runs in self.data.items():
            if (
                test_runs
                and isinstance(test_runs[0], dict)
                and "p99_ms" in test_runs[0]
            ):
                categories.append(test_name.replace("_", " ").title())
                # Take average across runs
                p99_vals = [run["p99_ms"] for run in test_runs if "p99_ms" in run]
                mean_vals = [
                    run.get("mean_ms", 0) for run in test_runs if "mean_ms" in run
                ]

                if p99_vals:
                    p99_values.append(statistics.mean(p99_vals))
                if mean_vals:
                    mean_values.append(statistics.mean(mean_vals))

        if categories:
            x = np.arange(len(categories))
            width = 0.35

            bars1 = ax.bar(
                x - width / 2,
                p99_values,
                width,
                label="P99 Latency",
                alpha=0.8,
                color="red",
            )
            bars2 = ax.bar(
                x + width / 2,
                mean_values,
                width,
                label="Mean Latency",
                alpha=0.8,
                color="blue",
            )

            ax.set_xlabel("Test Category")
            ax.set_ylabel("Latency (ms)")
            ax.set_title("Performance Overview: P99 vs Mean Latency")
            ax.set_xticks(x)
            ax.set_xticklabels(categories, rotation=45, ha="right")
            ax.legend()
            ax.grid(True, alpha=0.3)

            # Add value labels
            for bar in bars1:
                height = bar.get_height()
                ax.text(
                    bar.get_x() + bar.get_width() / 2.0,
                    height + 0.01,
                    ".2f",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                )

    def _plot_latency_distribution(self, ax):
        """Plot latency distribution histogram."""
        all_latencies = []

        for test_name, test_runs in self.data.items():
            for run in test_runs:
                if isinstance(run, dict):
                    # Try different latency fields
                    for field in ["p99_ms", "mean_ms", "p95_ms", "p50_ms"]:
                        if field in run:
                            all_latencies.append(run[field])
                            break

        if all_latencies:
            ax.hist(
                all_latencies, bins=20, alpha=0.7, edgecolor="black", color="skyblue"
            )
            ax.set_xlabel("Latency (ms)")
            ax.set_ylabel("Frequency")
            ax.set_title("Latency Distribution Across All Tests")
            ax.grid(True, alpha=0.3)

            # Add statistics
            mean_lat = statistics.mean(all_latencies)
            median_lat = statistics.median(all_latencies)
            ax.axvline(
                mean_lat,
                color="red",
                linestyle="--",
                alpha=0.8,
                label=f"Mean: {mean_lat:.2f}ms",
            )
            ax.axvline(
                median_lat,
                color="green",
                linestyle="--",
                alpha=0.8,
                label=f"Median: {median_lat:.2f}ms",
            )
            ax.legend()

    def _plot_stress_test_results(self, ax):
        """Plot stress test results."""
        stress_data = self.data.get("stress_tests", [])
        if not stress_data:
            ax.text(
                0.5,
                0.5,
                "No Stress Test Data Available",
                ha="center",
                va="center",
                transform=ax.transAxes,
            )
            ax.set_title("Stress Test Results")
            return

        # Extract stress test metrics
        categories = []
        values = []

        for stress_run in stress_data:
            if isinstance(stress_run, dict):
                for test_name, test_data in stress_run.items():
                    if isinstance(test_data, dict):
                        if "success" in test_data:
                            categories.append(test_name.replace("_", " ").title())
                            values.append(1 if test_data["success"] else 0)

        if categories and values:
            colors = ["green" if v == 1 else "red" for v in values]
            bars = ax.bar(categories, values, alpha=0.7, color=colors)
            ax.set_xlabel("Stress Test")
            ax.set_ylabel("Success (0/1)")
            ax.set_title("Stress Test Success Rates")
            ax.set_yticks([0, 1])
            ax.set_yticklabels(["Failed", "Passed"])
            ax.grid(True, alpha=0.3)

    def _plot_requirements_compliance(self, ax):
        """Plot requirements compliance chart."""
        # Define URC 2026 requirements
        requirements = {
            "Binary Protocol": {"required": 1.0, "actual": None},
            "IPC Bridge": {"required": 5.0, "actual": None},
            "Motion Control": {"required": 20.0, "actual": None},
            "End-to-End Pipeline": {"required": 5.0, "actual": None},
        }

        # Fill in actual results from data
        for test_name, test_runs in self.data.items():
            for run in test_runs:
                if isinstance(run, dict) and "p99_ms" in run:
                    p99_val = run["p99_ms"]
                    if "binary" in test_name.lower():
                        requirements["Binary Protocol"]["actual"] = p99_val
                    elif "ipc" in test_name.lower():
                        requirements["IPC Bridge"]["actual"] = p99_val
                    elif "motion" in test_name.lower():
                        requirements["Motion Control"]["actual"] = p99_val
                    elif (
                        "end_to_end" in test_name.lower()
                        or "pipeline" in test_name.lower()
                    ):
                        requirements["End-to-End Pipeline"]["actual"] = p99_val

        # Create compliance chart
        categories = list(requirements.keys())
        required = [req["required"] for req in requirements.values()]
        actual = [
            req["actual"] if req["actual"] is not None else 0
            for req in requirements.values()
        ]

        x = np.arange(len(categories))
        width = 0.35

        bars1 = ax.bar(
            x - width / 2,
            required,
            width,
            label="Required (ms)",
            alpha=0.8,
            color="red",
        )
        bars2 = ax.bar(
            x + width / 2, actual, width, label="Actual (ms)", alpha=0.8, color="green"
        )

        ax.set_xlabel("Performance Category")
        ax.set_ylabel("P99 Latency (ms)")
        ax.set_title("Requirements vs Actual Performance")
        ax.set_xticks(x)
        ax.set_xticklabels(categories, rotation=45, ha="right")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Add compliance indicators
        for i, (req, act) in enumerate(zip(required, actual)):
            if act > 0:
                compliance = "✅" if act <= req else "❌"
                ax.text(
                    x[i],
                    max(req, act) + 0.5,
                    compliance,
                    ha="center",
                    va="bottom",
                    fontsize=12,
                )

    def _plot_performance_trends(self, ax):
        """Plot performance trends over time."""
        # This would show trends if we had multiple runs over time
        # For now, show variability within single runs

        test_variability = {}
        for test_name, test_runs in self.data.items():
            latencies = []
            for run in test_runs:
                if isinstance(run, dict):
                    for field in ["p99_ms", "mean_ms", "p95_ms"]:
                        if field in run:
                            latencies.append(run[field])
                            break

            if len(latencies) > 1:
                test_variability[test_name] = {
                    "mean": statistics.mean(latencies),
                    "stdev": statistics.stdev(latencies) if len(latencies) > 1 else 0,
                    "cv": (
                        (statistics.stdev(latencies) / statistics.mean(latencies)) * 100
                        if len(latencies) > 1
                        else 0
                    ),
                }

        if test_variability:
            categories = list(test_variability.keys())
            means = [test_variability[cat]["mean"] for cat in categories]
            stdevs = [test_variability[cat]["stdev"] for cat in categories]

            x = np.arange(len(categories))
            bars = ax.bar(x, means, alpha=0.7, color="purple", yerr=stdevs, capsize=5)

            ax.set_xlabel("Test Category")
            ax.set_ylabel("Latency (ms)")
            ax.set_title("Performance Variability (Mean ± StdDev)")
            ax.set_xticks(x)
            ax.set_xticklabels(
                [cat.replace("_", " ").title() for cat in categories],
                rotation=45,
                ha="right",
            )
            ax.grid(True, alpha=0.3)

    def _plot_statistical_summary(self, ax):
        """Plot statistical summary of all performance data."""
        all_latencies = []
        all_categories = []

        for test_name, test_runs in self.data.items():
            for run in test_runs:
                if isinstance(run, dict):
                    for field in ["p99_ms", "mean_ms", "p95_ms", "p50_ms"]:
                        if field in run:
                            all_latencies.append(run[field])
                            all_categories.append(test_name.replace("_", " ").title())
                            break

        if all_latencies:
            # Create box plot
            data_dict = {}
            for cat, lat in zip(all_categories, all_latencies):
                if cat not in data_dict:
                    data_dict[cat] = []
                data_dict[cat].append(lat)

            categories = list(data_dict.keys())
            values = [data_dict[cat] for cat in categories]

            ax.boxplot(values, labels=categories)
            ax.set_ylabel("Latency (ms)")
            ax.set_title("Statistical Distribution by Test Category")
            ax.tick_params(axis="x", rotation=45)
            ax.grid(True, alpha=0.3)

    def generate_chaos_analysis_graphs(self):
        """Generate chaos engineering analysis graphs."""
        print("🎭 Generating Chaos Engineering Analysis...")

        # This would analyze chaos test results if available
        # For now, create a placeholder analysis

        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        ax.text(
            0.5,
            0.5,
            "Chaos Engineering Analysis\n\n(Not implemented in current test run)",
            ha="center",
            va="center",
            transform=ax.transAxes,
            fontsize=14,
        )
        ax.set_title("Chaos Engineering Impact Analysis")
        ax.axis("off")

        chaos_file = (
            self.output_dir / f"chaos_analysis_{int(datetime.now().timestamp())}.png"
        )
        plt.savefig(chaos_file, dpi=300, bbox_inches="tight")
        plt.close()

        print(f"🎭 Chaos analysis placeholder saved to: {chaos_file}")

    def generate_executive_summary_report(self):
        """Generate executive summary report."""
        print("📋 Generating Executive Summary Report...")

        summary = {
            "generated_at": datetime.now().isoformat(),
            "test_summary": {},
            "performance_highlights": {},
            "compliance_status": {},
            "recommendations": [],
        }

        # Test summary
        total_tests = len(self.data)
        successful_tests = 0
        total_measurements = 0

        for test_name, test_runs in self.data.items():
            for run in test_runs:
                total_measurements += 1
                if isinstance(run, dict) and run.get("success", False):
                    successful_tests += 1

        summary["test_summary"] = {
            "total_test_categories": total_tests,
            "total_measurements": total_measurements,
            "successful_measurements": successful_tests,
            "success_rate": (
                (successful_tests / total_measurements * 100)
                if total_measurements > 0
                else 0
            ),
        }

        # Performance highlights
        best_performers = []
        worst_performers = []

        for test_name, test_runs in self.data.items():
            latencies = []
            for run in test_runs:
                if isinstance(run, dict):
                    for field in ["p99_ms", "mean_ms"]:
                        if field in run:
                            latencies.append(run[field])
                            break

            if latencies:
                avg_latency = statistics.mean(latencies)
                if avg_latency < 1.0:
                    best_performers.append((test_name, avg_latency))
                elif avg_latency > 10.0:
                    worst_performers.append((test_name, avg_latency))

        summary["performance_highlights"] = {
            "best_performing_tests": best_performers[:3],
            "worst_performing_tests": worst_performers[:3],
            "overall_performance_trend": (
                "good"
                if len(best_performers) > len(worst_performers)
                else "needs_improvement"
            ),
        }

        # Compliance status
        compliance_checks = {
            "binary_protocol": {"required": 1.0, "actual": None},
            "ipc_bridge": {"required": 5.0, "actual": None},
            "motion_control": {"required": 20.0, "actual": None},
            "end_to_end": {"required": 5.0, "actual": None},
        }

        compliant_count = 0
        for test_name, test_runs in self.data.items():
            for run in test_runs:
                if isinstance(run, dict) and "p99_ms" in run:
                    p99_val = run["p99_ms"]
                    if "binary" in test_name.lower():
                        compliance_checks["binary_protocol"]["actual"] = p99_val
                        if p99_val <= 1.0:
                            compliant_count += 1
                    elif "ipc" in test_name.lower():
                        compliance_checks["ipc_bridge"]["actual"] = p99_val
                        if p99_val <= 5.0:
                            compliant_count += 1
                    elif "motion" in test_name.lower():
                        compliance_checks["motion_control"]["actual"] = p99_val
                        if p99_val <= 20.0:
                            compliant_count += 1
                    elif "end_to_end" in test_name.lower():
                        compliance_checks["end_to_end"]["actual"] = p99_val
                        if p99_val <= 5.0:
                            compliant_count += 1

        summary["compliance_status"] = {
            "total_requirements": len(compliance_checks),
            "compliant_requirements": compliant_count,
            "compliance_rate": (
                (compliant_count / len(compliance_checks) * 100)
                if compliance_checks
                else 0
            ),
            "details": compliance_checks,
        }

        # Generate recommendations
        recommendations = []

        compliance_rate = summary["compliance_status"]["compliance_rate"]
        if compliance_rate >= 80:
            recommendations.append(
                "✅ Excellent performance - system meets most requirements"
            )
        elif compliance_rate >= 60:
            recommendations.append("⚠️ Good performance - minor optimizations needed")
        else:
            recommendations.append(
                "❌ Performance issues detected - significant improvements required"
            )

        if worst_performers:
            recommendations.append(
                f"🔧 Focus optimization on: {', '.join([name for name, _ in worst_performers[:2]])}"
            )

        if successful_tests < total_measurements * 0.8:
            recommendations.append(
                "🧪 Increase test reliability and reduce measurement variability"
            )

        summary["recommendations"] = recommendations

        # Save executive summary
        summary_file = (
            self.output_dir
            / f"executive_summary_{int(datetime.now().timestamp())}.json"
        )
        with open(summary_file, "w") as f:
            json.dump(summary, f, indent=2, default=str)

        print(f"📋 Executive summary saved to: {summary_file}")
        return summary

    def print_executive_summary(self, summary: Dict[str, Any]):
        """Print human-readable executive summary."""
        print("\n" + "=" * 80)
        print("🎯 URC 2026 PERFORMANCE EXECUTIVE SUMMARY")
        print("=" * 80)

        # Test summary
        test_sum = summary.get("test_summary", {})
        print("\n📊 TEST SUMMARY:")
        print(f"  Categories Tested: {test_sum.get('total_test_categories', 0)}")
        print(f"  Total Measurements: {test_sum.get('total_measurements', 0)}")
        print(f"  Success Rate: {test_sum.get('success_rate', 0):.1f}%")
        # Performance highlights
        perf = summary.get("performance_highlights", {})
        best = perf.get("best_performing_tests", [])
        worst = perf.get("worst_performing_tests", [])

        if best:
            print("\n🏆 BEST PERFORMING TESTS:")
            for test_name, latency in best:
                print(".3f")
        if worst:
            print("\n⚠️ TESTS NEEDING ATTENTION:")
            for test_name, latency in worst:
                print(".3f")

        # Compliance status
        comp = summary.get("compliance_status", {})
        print("\n📏 COMPLIANCE STATUS:")
        print(
            f"  Requirements Met: {comp.get('compliant_requirements', 0)}/{comp.get('total_requirements', 0)}"
        )
        print(f"  Compliance Rate: {comp.get('compliance_rate', 0):.1f}%")
        # Recommendations
        recs = summary.get("recommendations", [])
        if recs:
            print("\n💡 RECOMMENDATIONS:")
            for rec in recs:
                print(f"  {rec}")

        print("\n" + "=" * 80)


def run_chaos_tests():
    """Run chaos engineering tests separately."""
    print("🎭 Running Chaos Engineering Tests...")

    try:
        import unittest
        from tests.chaos.test_chaos_engineering import (
            TestNetworkChaos,
            TestServiceChaos,
        )

        # Create test suite
        suite = unittest.TestSuite()

        # Add network chaos tests
        network_tests = unittest.TestLoader().loadTestsFromTestCase(TestNetworkChaos)
        suite.addTests(network_tests)

        # Add service chaos tests
        service_tests = unittest.TestLoader().loadTestsFromTestCase(TestServiceChaos)
        suite.addTests(service_tests)

        # Run tests
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)

        chaos_results = {
            "total_tests": result.testsRun,
            "passed": len(result.successes) if hasattr(result, "successes") else 0,
            "failed": len(result.failures),
            "errors": len(result.errors),
            "success_rate": (
                (len(result.successes) if hasattr(result, "successes") else 0)
                / result.testsRun
                * 100
                if result.testsRun > 0
                else 0
            ),
        }

        print("\n🎭 CHAOS TEST RESULTS:")
        print(f"  Total Tests: {chaos_results['total_tests']}")
        print(f"  Passed: {chaos_results['passed']}")
        print(f"  Failed: {chaos_results['failed']}")
        print(".1f")
        return chaos_results

    except Exception as e:
        print(f"❌ Chaos tests failed: {e}")
        return {"error": str(e)}


def main():
    """Main entry point for performance graphing and analysis."""
    parser = argparse.ArgumentParser(
        description="URC 2026 Performance Graphing and Analysis"
    )
    parser.add_argument(
        "--input-reports", nargs="+", help="Input performance report JSON files"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="performance_graphs",
        help="Output directory for graphs and reports (default: performance_graphs)",
    )
    parser.add_argument(
        "--find-reports",
        action="store_true",
        help="Automatically find all performance report files",
    )
    parser.add_argument(
        "--run-chaos-tests", action="store_true", help="Run chaos engineering tests"
    )
    parser.add_argument(
        "--generate-dashboard",
        action="store_true",
        help="Generate comprehensive dashboard",
    )

    args = parser.parse_args()

    # Find report files
    if args.find_reports:
        import glob

        report_files = glob.glob("performance_baseline_report_*.json")
        print(f"📁 Found {len(report_files)} performance report files")
    elif args.input_reports:
        report_files = args.input_reports
    else:
        # Default to latest report
        import glob

        report_files = glob.glob("performance_baseline_report_*.json")
        if report_files:
            # Sort by timestamp and take the latest
            report_files.sort(
                key=lambda x: x.split("_")[-1].replace(".json", ""), reverse=True
            )
            report_files = [report_files[0]]
        else:
            print("❌ No performance report files found. Run performance tests first.")
            return 1

    # Create graph generator
    generator = PerformanceGraphGenerator(args.output_dir)

    # Load data
    generator.load_performance_data(report_files)

    # Run chaos tests if requested
    if args.run_chaos_tests:
        chaos_results = run_chaos_tests()
        # Save chaos results
        chaos_file = (
            Path(args.output_dir)
            / f"chaos_test_results_{int(datetime.now().timestamp())}.json"
        )
        with open(chaos_file, "w") as f:
            json.dump(chaos_results, f, indent=2)

    # Generate graphs and analysis
    if args.generate_dashboard:
        generator.generate_comprehensive_dashboard()

    generator.generate_chaos_analysis_graphs()

    # Generate executive summary
    summary = generator.generate_executive_summary_report()
    generator.print_executive_summary(summary)

    print(f"\n📊 All results saved to: {args.output_dir}/")
    print("✅ Performance analysis complete!")


if __name__ == "__main__":
    main()
