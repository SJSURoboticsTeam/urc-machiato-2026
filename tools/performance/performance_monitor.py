#!/usr/bin/env python3
"""
Performance Monitoring System
Tracks performance trends and detects regressions for competition systems.
"""

import json
import time
import os
from pathlib import Path
from typing import Dict, List, Any, Optional
from datetime import datetime, timedelta
import numpy as np
import matplotlib.pyplot as plt


class PerformanceMonitor:
    """
    Monitors performance trends and detects regressions.

    Tracks key performance metrics:
    - Context evaluation performance
    - Policy engine performance
    - Memory usage trends
    - CPU usage patterns
    """

    def __init__(self, data_dir: str = "performance_data"):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(exist_ok=True)

        # Performance standards (from requirements)
        self.standards = {
            'context_evaluation': {
                'max_mean_time': 0.1,  # 100ms
                'max_variability': 0.5,  # 50% coefficient of variation
                'regression_threshold': 0.05  # 5% degradation
            },
            'policy_engine': {
                'max_mean_time': 0.05,  # 50ms for emergency response
                'max_variability': 0.5,
                'regression_threshold': 0.05
            }
        }

    def record_performance_run(self, benchmark_data: Dict[str, Any], run_id: Optional[str] = None) -> str:
        """Record a performance test run."""
        if run_id is None:
            run_id = f"run_{int(time.time())}"

        run_data = {
            'run_id': run_id,
            'timestamp': time.time(),
            'datetime': datetime.now().isoformat(),
            'benchmarks': benchmark_data.get('benchmarks', []),
            'metadata': {
                'pytest_version': benchmark_data.get('pytest_version'),
                'benchmark_version': benchmark_data.get('benchmark_version'),
                'hostname': benchmark_data.get('hostname')
            }
        }

        # Save run data
        run_file = self.data_dir / f"{run_id}.json"
        with open(run_file, 'w') as f:
            json.dump(run_data, f, indent=2)

        return run_id

    def analyze_performance_trends(self, days: int = 30) -> Dict[str, Any]:
        """Analyze performance trends over the specified period."""
        # Load recent performance data
        recent_runs = self._load_recent_runs(days)

        if not recent_runs:
            return {'error': f'No performance data found for the last {days} days'}

        # Analyze each benchmark
        analysis = {}
        for benchmark_name in self._get_unique_benchmarks(recent_runs):
            analysis[benchmark_name] = self._analyze_benchmark_trend(benchmark_name, recent_runs)

        # Overall assessment
        overall_assessment = self._calculate_overall_assessment(analysis)

        return {
            'period_days': days,
            'runs_analyzed': len(recent_runs),
            'benchmark_analysis': analysis,
            'overall_assessment': overall_assessment,
            'recommendations': self._generate_recommendations(analysis)
        }

    def detect_regressions(self, current_run_id: str, baseline_days: int = 7) -> Dict[str, Any]:
        """Detect performance regressions compared to baseline."""
        # Load current run
        current_file = self.data_dir / f"{current_run_id}.json"
        if not current_file.exists():
            return {'error': f'Current run {current_run_id} not found'}

        with open(current_file, 'r') as f:
            current_data = json.load(f)

        # Load baseline data
        baseline_runs = self._load_recent_runs(baseline_days)
        if not baseline_runs:
            return {'error': f'No baseline data found for the last {baseline_days} days'}

        # Compare each benchmark
        regressions = {}
        for benchmark in current_data.get('benchmarks', []):
            benchmark_name = benchmark['name'].split('::')[-1]
            current_stats = benchmark['stats']

            # Calculate baseline statistics
            baseline_times = []
            for run in baseline_runs:
                for b in run.get('benchmarks', []):
                    if b['name'].split('::')[-1] == benchmark_name:
                        baseline_times.extend(b.get('stats', {}).get('data', []))

            if baseline_times:
                baseline_mean = np.mean(baseline_times)
                baseline_std = np.std(baseline_times)
                current_mean = current_stats['mean']

                # Calculate regression
                regression_pct = (current_mean - baseline_mean) / baseline_mean if baseline_mean > 0 else 0
                std_deviations = abs(regression_pct) / (baseline_std / baseline_mean) if baseline_mean > 0 and baseline_std > 0 else 0

                regressions[benchmark_name] = {
                    'current_mean': current_mean,
                    'baseline_mean': baseline_mean,
                    'baseline_std': baseline_std,
                    'regression_pct': regression_pct,
                    'std_deviations': std_deviations,
                    'is_regression': regression_pct > self.standards.get(benchmark_name.split('_')[0], {}).get('regression_threshold', 0.05),
                    'severity': self._classify_regression_severity(regression_pct)
                }

        return {
            'current_run': current_run_id,
            'baseline_period_days': baseline_days,
            'baseline_runs': len(baseline_runs),
            'regressions': regressions,
            'summary': self._summarize_regressions(regressions)
        }

    def generate_performance_report(self, output_file: str, days: int = 30):
        """Generate comprehensive performance report."""
        trends = self.analyze_performance_trends(days)
        current_run_id = f"report_{int(time.time())}"

        # Create visualizations
        self._generate_performance_charts(days)

        report = {
            'generated_at': datetime.now().isoformat(),
            'report_period_days': days,
            'performance_trends': trends,
            'charts_generated': [
                'performance_trends.png',
                'regression_analysis.png'
            ]
        }

        with open(output_file, 'w') as f:
            json.dump(report, f, indent=2)

        return report

    def _load_recent_runs(self, days: int) -> List[Dict[str, Any]]:
        """Load performance runs from the last N days."""
        cutoff_time = time.time() - (days * 24 * 60 * 60)
        runs = []

        for run_file in self.data_dir.glob("*.json"):
            if run_file.name.startswith('run_'):
                try:
                    with open(run_file, 'r') as f:
                        run_data = json.load(f)

                    if run_data.get('timestamp', 0) > cutoff_time:
                        runs.append(run_data)
                except:
                    pass  # Skip corrupted files

        # Sort by timestamp
        runs.sort(key=lambda x: x.get('timestamp', 0))
        return runs

    def _get_unique_benchmarks(self, runs: List[Dict[str, Any]]) -> List[str]:
        """Get list of unique benchmark names."""
        benchmark_names = set()
        for run in runs:
            for benchmark in run.get('benchmarks', []):
                benchmark_names.add(benchmark['name'].split('::')[-1])
        return list(benchmark_names)

    def _analyze_benchmark_trend(self, benchmark_name: str, runs: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Analyze trend for a specific benchmark."""
        times = []
        timestamps = []

        for run in runs:
            for benchmark in run.get('benchmarks', []):
                if benchmark['name'].split('::')[-1] == benchmark_name:
                    times.extend(benchmark.get('stats', {}).get('data', []))
                    timestamps.append(run.get('timestamp', 0))

        if not times:
            return {'error': f'No data for benchmark {benchmark_name}'}

        # Calculate statistics
        mean_time = np.mean(times)
        std_time = np.std(times)
        min_time = np.min(times)
        max_time = np.max(times)

        # Trend analysis
        if len(timestamps) > 1:
            # Simple linear regression for trend
            x = np.array(range(len(timestamps)))
            y = np.array([np.mean([t for t in times[i:i+len(times)//len(timestamps)]]) for i in range(0, len(times), len(times)//len(timestamps))][:len(timestamps)])
            if len(x) == len(y):
                slope, intercept = np.polyfit(x, y, 1)
                trend_pct = slope / intercept if intercept != 0 else 0
            else:
                trend_pct = 0
        else:
            trend_pct = 0

        # Compliance check
        benchmark_type = benchmark_name.split('_')[0]
        standards = self.standards.get(benchmark_type, {})
        compliant = (mean_time <= standards.get('max_mean_time', float('inf')) and
                    (std_time / mean_time if mean_time > 0 else 0) <= standards.get('max_variability', 1.0))

        return {
            'mean_time': mean_time,
            'std_time': std_time,
            'min_time': min_time,
            'max_time': max_time,
            'trend_pct': trend_pct,
            'data_points': len(times),
            'compliant': compliant,
            'standard_max_time': standards.get('max_mean_time'),
            'standard_max_variability': standards.get('max_variability')
        }

    def _calculate_overall_assessment(self, analysis: Dict[str, Any]) -> Dict[str, Any]:
        """Calculate overall performance assessment."""
        total_benchmarks = len(analysis)
        compliant_benchmarks = sum(1 for b in analysis.values() if b.get('compliant', False))

        # Trend assessment
        improving_trends = sum(1 for b in analysis.values() if b.get('trend_pct', 0) < -0.01)  # >1% improvement
        degrading_trends = sum(1 for b in analysis.values() if b.get('trend_pct', 0) > 0.01)   # >1% degradation

        compliance_rate = compliant_benchmarks / total_benchmarks if total_benchmarks > 0 else 0

        return {
            'total_benchmarks': total_benchmarks,
            'compliant_benchmarks': compliant_benchmarks,
            'compliance_rate': compliance_rate,
            'improving_trends': improving_trends,
            'degrading_trends': degrading_trends,
            'overall_health': self._assess_overall_health(compliance_rate, degrading_trends)
        }

    def _assess_overall_health(self, compliance_rate: float, degrading_trends: int) -> str:
        """Assess overall performance health."""
        if compliance_rate >= 0.95 and degrading_trends == 0:
            return 'EXCELLENT'
        elif compliance_rate >= 0.90 and degrading_trends <= 1:
            return 'GOOD'
        elif compliance_rate >= 0.80:
            return 'FAIR'
        else:
            return 'CONCERNING'

    def _generate_recommendations(self, analysis: Dict[str, Any]) -> List[str]:
        """Generate performance improvement recommendations."""
        recommendations = []

        for benchmark_name, data in analysis.items():
            if not data.get('compliant', True):
                recommendations.append(f"Address performance issues in {benchmark_name}")

            trend = data.get('trend_pct', 0)
            if trend > 0.05:  # 5% degradation
                recommendations.append(f"Investigate performance degradation in {benchmark_name}")

        if not recommendations:
            recommendations.append("Performance is within acceptable limits")

        return recommendations

    def _classify_regression_severity(self, regression_pct: float) -> str:
        """Classify regression severity."""
        abs_pct = abs(regression_pct)
        if abs_pct < 0.01:  # <1%
            return 'minor'
        elif abs_pct < 0.05:  # <5%
            return 'moderate'
        else:  # >=5%
            return 'severe'

    def _summarize_regressions(self, regressions: Dict[str, Any]) -> Dict[str, Any]:
        """Summarize regression findings."""
        severe_regressions = [name for name, data in regressions.items()
                             if data.get('is_regression') and data.get('severity') == 'severe']
        moderate_regressions = [name for name, data in regressions.items()
                               if data.get('is_regression') and data.get('severity') == 'moderate']

        return {
            'total_regressions': len([r for r in regressions.values() if r.get('is_regression')]),
            'severe_regressions': len(severe_regressions),
            'moderate_regressions': len(moderate_regressions),
            'severe_regression_names': severe_regressions,
            'moderate_regression_names': moderate_regressions,
            'action_required': len(severe_regressions) > 0
        }

    def _generate_performance_charts(self, days: int):
        """Generate performance visualization charts."""
        try:
            runs = self._load_recent_runs(days)
            if not runs:
                return

            # Create trend chart
            plt.figure(figsize=(12, 8))

            for i, benchmark_name in enumerate(self._get_unique_benchmarks(runs)):
                times = []
                timestamps = []

                for run in runs:
                    for benchmark in run.get('benchmarks', []):
                        if benchmark['name'].split('::')[-1] == benchmark_name:
                            times.append(benchmark['stats']['mean'])
                            timestamps.append(datetime.fromtimestamp(run['timestamp']))

                if times:
                    plt.subplot(2, 2, i+1)
                    plt.plot(timestamps, times, marker='o', label=benchmark_name)
                    plt.title(f'{benchmark_name} Performance Trend')
                    plt.xlabel('Time')
                    plt.ylabel('Execution Time (s)')
                    plt.xticks(rotation=45)

            plt.tight_layout()
            plt.savefig(self.data_dir / 'performance_trends.png')
            plt.close()

        except Exception as e:
            print(f"Warning: Could not generate performance charts: {e}")


def main():
    """Command-line interface for performance monitoring."""
    import argparse

    parser = argparse.ArgumentParser(description='Performance Monitoring System')
    parser.add_argument('--record', help='Record a performance run from JSON file')
    parser.add_argument('--analyze', type=int, default=30, help='Analyze trends for N days')
    parser.add_argument('--detect-regressions', help='Detect regressions for run ID')
    parser.add_argument('--baseline-days', type=int, default=7, help='Baseline period in days')
    parser.add_argument('--report', help='Generate comprehensive report')

    args = parser.parse_args()

    monitor = PerformanceMonitor()

    if args.record:
        with open(args.record, 'r') as f:
            data = json.load(f)
        run_id = monitor.record_performance_run(data)
        print(f"Recorded performance run: {run_id}")

    elif args.detect_regressions:
        result = monitor.detect_regressions(args.detect_regressions, args.baseline_days)
        print(json.dumps(result, indent=2))

    elif args.report:
        report = monitor.generate_performance_report(args.report, args.analyze)
        print(f"Generated performance report: {args.report}")
        print(json.dumps(report, indent=2))

    else:
        # Default: analyze trends
        result = monitor.analyze_performance_trends(args.analyze)
        print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()


