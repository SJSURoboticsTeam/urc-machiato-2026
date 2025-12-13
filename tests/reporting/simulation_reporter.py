#!/usr/bin/env python3
"""
Simulation-Aware Test Reporter

Generates comprehensive reports with clear simulation vs hardware distinction.
Ensures stakeholders understand test validation status.

Author: URC 2026 Autonomy Team
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class SimulationTestReport:
    """Generate simulation-aware test reports with clear warnings."""

    def __init__(self, output_dir: Path):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.results: List[Dict[str, Any]] = []

    def add_result(self, result: Dict[str, Any]):
        """Add a test result to the report."""
        # Ensure simulation metadata is present
        if "simulation_metadata" not in result:
            result["simulation_metadata"] = {
                "is_simulation": True,
                "hardware_validated": False,
                "simulation_confidence": "MEDIUM",
            }

        # Add simulation warning if passing
        if result.get("status") == "PASSED":
            if "warnings" not in result:
                result["warnings"] = []

            env_tier = result.get("environment_tier", "UNKNOWN")
            net_profile = result.get("network_profile", "UNKNOWN")

            result["warnings"].append(
                f"⚠️ {env_tier.upper()} SIMULATION PASS "
                f"(network: {net_profile}) - Hardware validation required"
            )

        self.results.append(result)

    def generate_json_report(self) -> Path:
        """Generate JSON report with simulation warnings."""
        report = {
            "generated_at": datetime.now().isoformat(),
            "report_type": "SIMULATION_TEST_REPORT",
            "simulation_warning": "🚨 ALL TESTS ARE SIMULATION-BASED - Hardware validation required before deployment",
            "summary": self._generate_summary(),
            "by_environment_tier": self._aggregate_by_tier(),
            "by_network_profile": self._aggregate_by_network(),
            "validation_status": self._generate_validation_status(),
            "test_results": self.results,
            "warnings": self._collect_warnings(),
        }

        output_file = self.output_dir / "simulation_test_report.json"
        with open(output_file, "w") as f:
            json.dump(report, f, indent=2)

        return output_file

    def generate_html_report(self) -> Path:
        """Generate HTML report with visual indicators."""
        report_data = self._load_or_generate_json()

        html = self._generate_html_template(report_data)

        output_file = self.output_dir / "simulation_test_report.html"
        with open(output_file, "w") as f:
            f.write(html)

        return output_file

    def generate_markdown_summary(self) -> Path:
        """Generate markdown summary for GitHub/documentation."""
        report_data = self._load_or_generate_json()

        md = self._generate_markdown_template(report_data)

        output_file = self.output_dir / "simulation_test_summary.md"
        with open(output_file, "w") as f:
            f.write(md)

        return output_file

    def _generate_summary(self) -> Dict[str, Any]:
        """Generate test summary statistics."""
        total = len(self.results)
        passed = sum(1 for r in self.results if r.get("status") == "PASSED")
        failed = sum(1 for r in self.results if r.get("status") == "FAILED")
        skipped = sum(1 for r in self.results if r.get("status") == "SKIPPED")

        return {
            "total_tests": total,
            "passed": passed,
            "failed": failed,
            "skipped": skipped,
            "pass_rate": (passed / total * 100) if total > 0 else 0,
        }

    def _aggregate_by_tier(self) -> Dict[str, Dict[str, int]]:
        """Aggregate results by environment tier."""
        tiers = {}

        for result in self.results:
            tier = result.get("environment_tier", "UNKNOWN")
            if tier not in tiers:
                tiers[tier] = {"total": 0, "passed": 0, "failed": 0}

            tiers[tier]["total"] += 1
            if result.get("status") == "PASSED":
                tiers[tier]["passed"] += 1
            elif result.get("status") == "FAILED":
                tiers[tier]["failed"] += 1

        return tiers

    def _aggregate_by_network(self) -> Dict[str, Dict[str, int]]:
        """Aggregate results by network profile."""
        profiles = {}

        for result in self.results:
            profile = result.get("network_profile", "UNKNOWN")
            if profile not in profiles:
                profiles[profile] = {"total": 0, "passed": 0, "failed": 0}

            profiles[profile]["total"] += 1
            if result.get("status") == "PASSED":
                profiles[profile]["passed"] += 1
            elif result.get("status") == "FAILED":
                profiles[profile]["failed"] += 1

        return profiles

    def _generate_validation_status(self) -> Dict[str, Any]:
        """Generate hardware validation status."""
        total = len(self.results)
        hw_validated = sum(
            1
            for r in self.results
            if r.get("simulation_metadata", {}).get("hardware_validated", False)
        )

        return {
            "total_tests": total,
            "hardware_validated": hw_validated,
            "simulation_only": total - hw_validated,
            "hardware_coverage_percent": (hw_validated / total * 100) if total > 0 else 0,
            "ready_for_production": hw_validated >= total * 0.8,
            "status": self._determine_validation_status(hw_validated, total),
        }

    def _determine_validation_status(self, hw_validated: int, total: int) -> str:
        """Determine overall validation status."""
        if total == 0:
            return "NO_TESTS"

        hw_percent = hw_validated / total * 100

        if hw_percent >= 80:
            return "PRODUCTION_READY"
        elif hw_percent >= 50:
            return "HARDWARE_VALIDATION_IN_PROGRESS"
        elif hw_percent >= 20:
            return "LIMITED_HARDWARE_VALIDATION"
        else:
            return "SIMULATION_ONLY"

    def _collect_warnings(self) -> List[str]:
        """Collect all unique warnings."""
        warnings = set()
        warnings.add(
            "🚨 ALL TESTS ARE SIMULATION-BASED - Hardware validation required before deployment"
        )

        for result in self.results:
            for warning in result.get("warnings", []):
                warnings.add(warning)

        return sorted(list(warnings))

    def _load_or_generate_json(self) -> Dict[str, Any]:
        """Load existing JSON report or generate new one."""
        json_file = self.output_dir / "simulation_test_report.json"

        if json_file.exists():
            with open(json_file, "r") as f:
                return json.load(f)
        else:
            self.generate_json_report()
            with open(json_file, "r") as f:
                return json.load(f)

    def _generate_html_template(self, report_data: Dict[str, Any]) -> str:
        """Generate HTML report template."""
        summary = report_data["summary"]
        validation = report_data["validation_status"]

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>URC 2026 Simulation Test Report</title>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 20px;
        }}
        .warning-banner {{
            background-color: #fff3cd;
            border-left: 5px solid #ffc107;
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
        }}
        .critical-warning {{
            background-color: #f8d7da;
            border-left: 5px solid #dc3545;
            padding: 15px;
            margin: 20px 0;
            border-radius: 5px;
        }}
        .summary-card {{
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin: 20px 0;
        }}
        .metric {{
            display: inline-block;
            margin: 10px 20px;
        }}
        .metric-value {{
            font-size: 2em;
            font-weight: bold;
            color: #667eea;
        }}
        .metric-label {{
            color: #666;
            font-size: 0.9em;
        }}
        .tier-section {{
            background: white;
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        .perfect {{ border-left: 4px solid #28a745; }}
        .real-life {{ border-left: 4px solid #ffc107; }}
        .extreme {{ border-left: 4px solid #dc3545; }}
        .validation-status {{
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
        }}
        .simulation-only {{
            background-color: #f8d7da;
            border: 2px solid #dc3545;
        }}
        .production-ready {{
            background-color: #d4edda;
            border: 2px solid #28a745;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin: 20px 0;
        }}
        th, td {{
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }}
        th {{
            background-color: #667eea;
            color: white;
        }}
        .status-passed {{ color: #28a745; font-weight: bold; }}
        .status-failed {{ color: #dc3545; font-weight: bold; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🧪 URC 2026 Simulation Test Report</h1>
        <p>Generated: {report_data['generated_at']}</p>
    </div>

    <div class="critical-warning">
        <h3>🚨 CRITICAL WARNING</h3>
        <p><strong>{report_data['simulation_warning']}</strong></p>
    </div>

    <div class="summary-card">
        <h2>📊 Test Summary</h2>
        <div class="metric">
            <div class="metric-value">{summary['total_tests']}</div>
            <div class="metric-label">Total Tests</div>
        </div>
        <div class="metric">
            <div class="metric-value" style="color: #28a745;">{summary['passed']}</div>
            <div class="metric-label">Passed</div>
        </div>
        <div class="metric">
            <div class="metric-value" style="color: #dc3545;">{summary['failed']}</div>
            <div class="metric-label">Failed</div>
        </div>
        <div class="metric">
            <div class="metric-value">{summary['pass_rate']:.1f}%</div>
            <div class="metric-label">Pass Rate</div>
        </div>
    </div>

    <div class="validation-status {validation['status'].lower().replace('_', '-')}">
        <h2>✅ Hardware Validation Status</h2>
        <p><strong>Status: {validation['status'].replace('_', ' ').title()}</strong></p>
        <p>Hardware Validated: {validation['hardware_validated']} / {validation['total_tests']}
           ({validation['hardware_coverage_percent']:.1f}%)</p>
        <p>Simulation Only: {validation['simulation_only']} tests</p>
        {"<p style='color: #28a745;'><strong>✅ Ready for production deployment</strong></p>" if validation['ready_for_production'] else "<p style='color: #dc3545;'><strong>❌ NOT ready for production - more hardware validation needed</strong></p>"}
    </div>

    <h2>🌍 Results by Environment Tier</h2>
    {self._generate_tier_html(report_data['by_environment_tier'])}

    <h2>📡 Results by Network Profile</h2>
    {self._generate_network_html(report_data['by_network_profile'])}

    <h2>⚠️ Warnings and Notices</h2>
    <ul>
        {"".join(f"<li>{warning}</li>" for warning in report_data['warnings'][:10])}
    </ul>

    <h2>📋 Detailed Results</h2>
    <table>
        <tr>
            <th>Test Name</th>
            <th>Status</th>
            <th>Environment</th>
            <th>Network</th>
            <th>Duration</th>
        </tr>
        {self._generate_results_table_html(report_data['test_results'])}
    </table>
</body>
</html>"""
        return html

    def _generate_tier_html(self, tier_data: Dict[str, Dict[str, int]]) -> str:
        """Generate HTML for tier breakdown."""
        html = ""
        tier_classes = {
            "perfect": "perfect",
            "real_life": "real-life",
            "extreme": "extreme",
        }

        for tier, data in tier_data.items():
            tier_class = tier_classes.get(tier.lower(), "")
            pass_rate = (data["passed"] / data["total"] * 100) if data["total"] > 0 else 0

            html += f"""
    <div class="tier-section {tier_class}">
        <h3>{tier.replace('_', ' ').title()}</h3>
        <p>Tests: {data['total']} | Passed: {data['passed']} | Failed: {data['failed']} | Pass Rate: {pass_rate:.1f}%</p>
    </div>"""

        return html

    def _generate_network_html(self, network_data: Dict[str, Dict[str, int]]) -> str:
        """Generate HTML for network profile breakdown."""
        html = "<div class='tier-section'>"

        for profile, data in network_data.items():
            pass_rate = (data["passed"] / data["total"] * 100) if data["total"] > 0 else 0

            html += f"""
        <p><strong>{profile.replace('_', ' ').title()}</strong>:
           {data['passed']}/{data['total']} passed ({pass_rate:.1f}%)</p>"""

        html += "</div>"
        return html

    def _generate_results_table_html(self, results: List[Dict[str, Any]]) -> str:
        """Generate HTML table rows for results."""
        html = ""

        for result in results[:50]:  # Limit to first 50 for performance
            status = result.get("status", "UNKNOWN")
            status_class = f"status-{status.lower()}"

            html += f"""
        <tr>
            <td>{result.get('test_name', 'Unknown')}</td>
            <td class="{status_class}">{status}</td>
            <td>{result.get('environment_tier', 'N/A')}</td>
            <td>{result.get('network_profile', 'N/A')}</td>
            <td>{result.get('duration_ms', 0):.1f}ms</td>
        </tr>"""

        if len(results) > 50:
            html += f"""
        <tr>
            <td colspan="5" style="text-align: center;">
                ... and {len(results) - 50} more results (see JSON report)
            </td>
        </tr>"""

        return html

    def _generate_markdown_template(self, report_data: Dict[str, Any]) -> str:
        """Generate markdown summary."""
        summary = report_data["summary"]
        validation = report_data["validation_status"]

        md = f"""# URC 2026 Simulation Test Report

**Generated**: {report_data['generated_at']}

## 🚨 CRITICAL WARNING

{report_data['simulation_warning']}

## 📊 Test Summary

- **Total Tests**: {summary['total_tests']}
- **Passed**: {summary['passed']}
- **Failed**: {summary['failed']}
- **Pass Rate**: {summary['pass_rate']:.1f}%

## ✅ Hardware Validation Status

- **Status**: {validation['status'].replace('_', ' ').title()}
- **Hardware Validated**: {validation['hardware_validated']} / {validation['total_tests']} ({validation['hardware_coverage_percent']:.1f}%)
- **Simulation Only**: {validation['simulation_only']} tests
- **Production Ready**: {'✅ Yes' if validation['ready_for_production'] else '❌ No'}

## 🌍 Results by Environment Tier

"""

        for tier, data in report_data["by_environment_tier"].items():
            pass_rate = (data["passed"] / data["total"] * 100) if data["total"] > 0 else 0
            md += f"- **{tier.replace('_', ' ').title()}**: {data['passed']}/{data['total']} passed ({pass_rate:.1f}%)\n"

        md += "\n## ⚠️ Warnings\n\n"
        for warning in report_data["warnings"][:5]:
            md += f"- {warning}\n"

        return md

    def print_summary(self):
        """Print human-readable summary to console."""
        report_data = self._load_or_generate_json()

        print("\n" + "=" * 70)
        print("🧪 SIMULATION TEST REPORT")
        print("=" * 70)
        print(f"\n🚨 {report_data['simulation_warning']}")
        print("\n" + "=" * 70)

        summary = report_data["summary"]
        print(f"\n📊 Summary: {summary['passed']}/{summary['total_tests']} passed ({summary['pass_rate']:.1f}%)")

        validation = report_data["validation_status"]
        print(f"\n✅ Hardware Validation: {validation['hardware_validated']}/{validation['total_tests']} ")
        print(f"   Status: {validation['status'].replace('_', ' ').title()}")
        print(f"   Production Ready: {'✅ Yes' if validation['ready_for_production'] else '❌ No'}")

        print("\n" + "=" * 70)


if __name__ == "__main__":
    # Test reporter
    print("📊 Testing Simulation Reporter")

    reporter = SimulationTestReport(Path("test_reports"))

    # Add sample results
    sample_results = [
        {
            "test_name": "test_gps_perfect",
            "status": "PASSED",
            "environment_tier": "perfect",
            "network_profile": "perfect",
            "duration_ms": 125.3,
        },
        {
            "test_name": "test_gps_real_life",
            "status": "PASSED",
            "environment_tier": "real_life",
            "network_profile": "rural_wifi",
            "duration_ms": 234.7,
        },
        {
            "test_name": "test_gps_extreme",
            "status": "FAILED",
            "environment_tier": "extreme",
            "network_profile": "extreme",
            "duration_ms": 1532.1,
        },
    ]

    for result in sample_results:
        reporter.add_result(result)

    # Generate reports
    json_file = reporter.generate_json_report()
    html_file = reporter.generate_html_report()
    md_file = reporter.generate_markdown_summary()

    print(f"✅ JSON report: {json_file}")
    print(f"✅ HTML report: {html_file}")
    print(f"✅ Markdown summary: {md_file}")

    reporter.print_summary()
