#!/usr/bin/env python3
"""
Calibration Workflow - Automated Calibration and Setup for Hardware Integration

Provides automated calibration workflows for cameras, sensors, and actuators
to ensure accurate operation during hardware integration.

Author: URC 2026 Autonomy Team
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


class CalibrationType(Enum):
    """Types of calibration procedures."""

    CAMERA_INTRINSICS = "camera_intrinsics"
    CAMERA_EXTRINSICS = "camera_extrinsics"
    ARM_KINEMATICS = "arm_kinematics"
    IMU_ALIGNMENT = "imu_alignment"
    GPS_OFFSET = "gps_offset"
    LIDAR_EXTRINSICS = "lidar_extrinsics"
    WHEEL_ODOMETRY = "wheel_odometry"


@dataclass
class CalibrationResult:
    """Result of a calibration procedure."""

    success: bool
    calibration_type: CalibrationType
    parameters: Dict[str, Any] = field(default_factory=dict)
    error_message: Optional[str] = None
    timestamp: float = field(default_factory=time.time)
    duration: float = 0.0
    quality_score: float = 0.0  # 0.0 to 1.0


@dataclass
class CalibrationStep:
    """Individual step in a calibration procedure."""

    name: str
    description: str
    function: Callable
    required_hardware: List[str] = field(default_factory=list)
    estimated_duration: float = 30.0  # seconds
    can_fail_safely: bool = True


class CalibrationWorkflow:
    """
    Automated calibration workflow system.

    Manages calibration procedures for all rover subsystems,
    ensuring proper setup before hardware integration.
    """

    def __init__(self, calibration_dir: str = "calibration_data"):
        """
        Initialize calibration workflow.

        Args:
            calibration_dir: Directory to store calibration results
        """
        self.calibration_dir = Path(calibration_dir)
        self.calibration_dir.mkdir(exist_ok=True)

        self.calibration_results: Dict[CalibrationType, CalibrationResult] = {}
        self.workflow_definitions = self._define_workflows()

        # Load existing calibration results
        self._load_existing_calibrations()

        logger.info("Calibration workflow initialized")

    def _define_workflows(self) -> Dict[CalibrationType, List[CalibrationStep]]:
        """Define calibration workflows for each calibration type."""
        return {
            CalibrationType.CAMERA_INTRINSICS: [
                CalibrationStep(
                    name="capture_checkerboard",
                    description="Capture checkerboard images from multiple angles",
                    function=self._capture_checkerboard_images,
                    required_hardware=["camera"],
                    estimated_duration=120.0,
                ),
                CalibrationStep(
                    name="detect_corners",
                    description="Detect checkerboard corners in images",
                    function=self._detect_checkerboard_corners,
                    estimated_duration=30.0,
                ),
                CalibrationStep(
                    name="compute_intrinsics",
                    description="Compute camera intrinsic parameters",
                    function=self._compute_camera_intrinsics,
                    estimated_duration=60.0,
                ),
                CalibrationStep(
                    name="validate_calibration",
                    description="Validate calibration quality",
                    function=self._validate_camera_intrinsics,
                    estimated_duration=15.0,
                ),
            ],
            CalibrationType.CAMERA_EXTRINSICS: [
                CalibrationStep(
                    name="capture_extrinsics_data",
                    description="Capture camera poses relative to robot base",
                    function=self._capture_extrinsics_data,
                    required_hardware=["camera", "arm"],
                    estimated_duration=180.0,
                ),
                CalibrationStep(
                    name="compute_extrinsics",
                    description="Compute camera extrinsic parameters",
                    function=self._compute_camera_extrinsics,
                    estimated_duration=45.0,
                ),
            ],
            CalibrationType.ARM_KINEMATICS: [
                CalibrationStep(
                    name="record_joint_positions",
                    description="Record joint angles at known poses",
                    function=self._record_arm_joint_positions,
                    required_hardware=["arm"],
                    estimated_duration=300.0,
                ),
                CalibrationStep(
                    name="compute_dh_parameters",
                    description="Compute Denavit-Hartenberg parameters",
                    function=self._compute_dh_parameters,
                    estimated_duration=120.0,
                ),
                CalibrationStep(
                    name="validate_kinematics",
                    description="Validate forward and inverse kinematics",
                    function=self._validate_arm_kinematics,
                    estimated_duration=60.0,
                ),
            ],
            CalibrationType.IMU_ALIGNMENT: [
                CalibrationStep(
                    name="collect_static_data",
                    description="Collect IMU data while rover is stationary",
                    function=self._collect_imu_static_data,
                    required_hardware=["imu"],
                    estimated_duration=60.0,
                ),
                CalibrationStep(
                    name="compute_alignment",
                    description="Compute IMU-to-robot alignment",
                    function=self._compute_imu_alignment,
                    estimated_duration=30.0,
                ),
            ],
            CalibrationType.GPS_OFFSET: [
                CalibrationStep(
                    name="collect_gps_data",
                    description="Collect GPS data at known positions",
                    function=self._collect_gps_offset_data,
                    required_hardware=["gps"],
                    estimated_duration=600.0,  # 10 minutes
                ),
                CalibrationStep(
                    name="compute_offset",
                    description="Compute GPS antenna offset",
                    function=self._compute_gps_offset,
                    estimated_duration=30.0,
                ),
            ],
        }

    def run_calibration(
        self, calibration_type: CalibrationType, force_redo: bool = False
    ) -> CalibrationResult:
        """
        Run a complete calibration workflow.

        Args:
            calibration_type: Type of calibration to perform
            force_redo: Force re-calibration even if already done

        Returns:
            CalibrationResult with success status and parameters
        """
        start_time = time.time()

        # Check if already calibrated
        if (
            not force_redo
            and calibration_type in self.calibration_results
            and self.calibration_results[calibration_type].success
        ):
            logger.info(f"Calibration {calibration_type.value} already completed")
            return self.calibration_results[calibration_type]

        logger.info(f"Starting calibration: {calibration_type.value}")

        # Get workflow steps
        if calibration_type not in self.workflow_definitions:
            error_msg = f"Unknown calibration type: {calibration_type}"
            logger.error(error_msg)
            return CalibrationResult(
                success=False,
                calibration_type=calibration_type,
                error_message=error_msg,
                duration=time.time() - start_time,
            )

        workflow = self.workflow_definitions[calibration_type]
        results = {}

        # Execute each step
        for step in workflow:
            logger.info(f"Executing step: {step.name} - {step.description}")

            try:
                step_result = step.function(calibration_type, results)
                results[step.name] = step_result

                if not step_result.get("success", False):
                    error_msg = f"Step {step.name} failed: {step_result.get('error', 'Unknown error')}"
                    logger.error(error_msg)

                    if not step.can_fail_safely:
                        return CalibrationResult(
                            success=False,
                            calibration_type=calibration_type,
                            error_message=error_msg,
                            duration=time.time() - start_time,
                        )
                    # Continue with other steps if this one can fail safely
                    continue

            except Exception as e:
                error_msg = f"Step {step.name} exception: {e}"
                logger.error(error_msg)

                if not step.can_fail_safely:
                    return CalibrationResult(
                        success=False,
                        calibration_type=calibration_type,
                        error_message=error_msg,
                        duration=time.time() - start_time,
                    )

        # Combine results into final calibration
        final_result = self._combine_calibration_results(calibration_type, results)
        final_result.duration = time.time() - start_time

        # Save calibration result
        self.calibration_results[calibration_type] = final_result
        self._save_calibration_result(final_result)

        if final_result.success:
            logger.info(
                f"✅ Calibration {calibration_type.value} completed successfully"
            )
        else:
            logger.error(f"❌ Calibration {calibration_type.value} failed")

        return final_result

    def run_full_calibration_suite(
        self, force_redo: bool = False
    ) -> Dict[CalibrationType, CalibrationResult]:
        """
        Run all available calibrations.

        Args:
            force_redo: Force re-calibration of all procedures

        Returns:
            Dict mapping calibration types to their results
        """
        results = {}

        for cal_type in self.workflow_definitions.keys():
            results[cal_type] = self.run_calibration(cal_type, force_redo)

        return results

    def get_calibration_status(self) -> Dict[CalibrationType, Dict[str, Any]]:
        """Get status of all calibrations."""
        status = {}

        for cal_type in CalibrationType:
            if cal_type in self.calibration_results:
                result = self.calibration_results[cal_type]
                status[cal_type] = {
                    "completed": True,
                    "success": result.success,
                    "quality_score": result.quality_score,
                    "timestamp": result.timestamp,
                    "duration": result.duration,
                }
            else:
                status[cal_type] = {
                    "completed": False,
                    "success": False,
                    "quality_score": 0.0,
                    "timestamp": None,
                    "duration": 0.0,
                }

        return status

    def validate_calibration_quality(
        self, calibration_type: CalibrationType
    ) -> Dict[str, Any]:
        """
        Validate the quality of an existing calibration.

        Args:
            calibration_type: Type of calibration to validate

        Returns:
            Dict with validation results
        """
        if calibration_type not in self.calibration_results:
            return {"valid": False, "error": "Calibration not found"}

        result = self.calibration_results[calibration_type]
        if not result.success:
            return {"valid": False, "error": "Calibration was not successful"}

        # Run quality validation based on type
        if calibration_type == CalibrationType.CAMERA_INTRINSICS:
            return self._validate_camera_intrinsics_quality(result.parameters)
        elif calibration_type == CalibrationType.ARM_KINEMATICS:
            return self._validate_arm_kinematics_quality(result.parameters)

        return {"valid": True, "quality_score": result.quality_score}

    def _capture_checkerboard_images(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Capture checkerboard images for camera calibration."""
        try:
            # This would integrate with actual camera hardware
            # For now, return mock success
            return {"success": True, "images_captured": 20, "checkerboard_size": (9, 6)}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _detect_checkerboard_corners(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Detect checkerboard corners in captured images."""
        try:
            # This would use OpenCV to detect corners
            return {"success": True, "corners_detected": 18, "images_used": 18}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _compute_camera_intrinsics(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Compute camera intrinsic parameters."""
        try:
            # This would use OpenCV calibrateCamera
            return {
                "success": True,
                "camera_matrix": [[1000, 0, 640], [0, 1000, 360], [0, 0, 1]],
                "distortion_coefficients": [0.1, -0.2, 0.001, 0.001, 0.0],
                "reprojection_error": 0.5,
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _validate_camera_intrinsics(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Validate camera intrinsics calibration."""
        try:
            intrinsics = previous_results.get("compute_intrinsics", {})
            reprojection_error = intrinsics.get("reprojection_error", 1.0)

            quality_score = max(
                0.0, 1.0 - reprojection_error
            )  # Lower error = higher quality

            return {
                "success": quality_score > 0.7,
                "quality_score": quality_score,
                "reprojection_error": reprojection_error,
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _capture_extrinsics_data(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Capture camera extrinsics data."""
        try:
            return {"success": True, "poses_captured": 15, "arm_positions": 15}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _compute_camera_extrinsics(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Compute camera extrinsic parameters."""
        try:
            return {
                "success": True,
                "rotation_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "translation_vector": [0.1, 0.0, 0.2],
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _record_arm_joint_positions(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Record arm joint positions for kinematics calibration."""
        try:
            return {"success": True, "poses_recorded": 25, "joints_per_pose": 6}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _compute_dh_parameters(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Compute Denavit-Hartenberg parameters."""
        try:
            return {
                "success": True,
                "dh_parameters": [
                    {"a": 0, "alpha": 0, "d": 0.1, "theta": 0},
                    {"a": 0.3, "alpha": 1.57, "d": 0, "theta": 0},
                    # ... more joints
                ],
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _validate_arm_kinematics(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Validate arm kinematics calibration."""
        try:
            return {
                "success": True,
                "position_error_mean": 0.005,  # 5mm
                "orientation_error_mean": 0.02,  # 1.15 degrees
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _collect_imu_static_data(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Collect static IMU data."""
        try:
            return {"success": True, "samples_collected": 1000, "duration_seconds": 60}
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _compute_imu_alignment(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Compute IMU alignment."""
        try:
            return {
                "success": True,
                "alignment_matrix": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "bias_vector": [0.01, -0.02, 0.005],
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _collect_gps_offset_data(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Collect GPS offset data."""
        try:
            return {
                "success": True,
                "positions_collected": 10,
                "total_distance": 50.0,  # meters
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _compute_gps_offset(
        self, cal_type: CalibrationType, previous_results: Dict
    ) -> Dict[str, Any]:
        """Compute GPS offset."""
        try:
            return {
                "success": True,
                "antenna_offset": [0.5, 0.0, 1.2],  # x, y, z in meters
                "accuracy_estimate": 0.05,  # meters
            }
        except Exception as e:
            return {"success": False, "error": str(e)}

    def _combine_calibration_results(
        self, cal_type: CalibrationType, step_results: Dict[str, Dict]
    ) -> CalibrationResult:
        """Combine step results into final calibration result."""
        # Extract parameters from successful steps
        parameters = {}
        quality_score = 0.0
        success = True

        for step_name, result in step_results.items():
            if result.get("success", False):
                # Merge parameters
                for key, value in result.items():
                    if key != "success":
                        parameters[f"{step_name}_{key}"] = value

                # Accumulate quality score
                if "quality_score" in result:
                    quality_score += result["quality_score"]
                else:
                    quality_score += 0.1  # Small bonus for successful step
            else:
                success = False
                break

        # Normalize quality score
        quality_score = min(1.0, quality_score / len(step_results))

        return CalibrationResult(
            success=success,
            calibration_type=cal_type,
            parameters=parameters,
            quality_score=quality_score,
        )

    def _save_calibration_result(self, result: CalibrationResult):
        """Save calibration result to disk."""
        filename = f"{result.calibration_type.value}_{int(result.timestamp)}.json"
        filepath = self.calibration_dir / filename

        data = {
            "calibration_type": result.calibration_type.value,
            "success": result.success,
            "parameters": result.parameters,
            "timestamp": result.timestamp,
            "duration": result.duration,
            "quality_score": result.quality_score,
            "error_message": result.error_message,
        }

        try:
            import json

            with open(filepath, "w") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            logger.error(f"Failed to save calibration result: {e}")

    def _load_existing_calibrations(self):
        """Load existing calibration results from disk."""
        try:
            import json

            for json_file in self.calibration_dir.glob("*.json"):
                with open(json_file, "r") as f:
                    data = json.load(f)

                cal_type = CalibrationType(data["calibration_type"])
                result = CalibrationResult(
                    success=data["success"],
                    calibration_type=cal_type,
                    parameters=data["parameters"],
                    timestamp=data["timestamp"],
                    duration=data["duration"],
                    quality_score=data.get("quality_score", 0.0),
                    error_message=data.get("error_message"),
                )

                self.calibration_results[cal_type] = result

        except Exception as e:
            logger.error(f"Failed to load existing calibrations: {e}")

    def _validate_camera_intrinsics_quality(self, parameters: Dict) -> Dict[str, Any]:
        """Validate camera intrinsics calibration quality."""
        reprojection_error = parameters.get(
            "compute_intrinsics_reprojection_error", 1.0
        )

        if reprojection_error < 0.5:
            return {
                "valid": True,
                "quality_score": 1.0,
                "message": "Excellent calibration",
            }
        elif reprojection_error < 1.0:
            return {"valid": True, "quality_score": 0.8, "message": "Good calibration"}
        elif reprojection_error < 2.0:
            return {
                "valid": True,
                "quality_score": 0.6,
                "message": "Acceptable calibration",
            }
        else:
            return {
                "valid": False,
                "quality_score": 0.0,
                "message": "Poor calibration - redo required",
            }

    def _validate_arm_kinematics_quality(self, parameters: Dict) -> Dict[str, Any]:
        """Validate arm kinematics calibration quality."""
        position_error = parameters.get("validate_kinematics_position_error_mean", 1.0)
        orientation_error = parameters.get(
            "validate_kinematics_orientation_error_mean", 1.0
        )

        # Convert orientation error to degrees for easier interpretation
        orientation_error_deg = orientation_error * 180.0 / 3.14159

        if position_error < 0.005 and orientation_error_deg < 1.0:  # 5mm, 1 degree
            return {
                "valid": True,
                "quality_score": 1.0,
                "message": "Excellent calibration",
            }
        elif position_error < 0.01 and orientation_error_deg < 2.0:
            return {"valid": True, "quality_score": 0.8, "message": "Good calibration"}
        elif position_error < 0.02 and orientation_error_deg < 5.0:
            return {
                "valid": True,
                "quality_score": 0.6,
                "message": "Acceptable calibration",
            }
        else:
            return {
                "valid": False,
                "quality_score": 0.0,
                "message": "Poor calibration - redo required",
            }


# Global calibration workflow instance
_workflow = None


def get_calibration_workflow() -> CalibrationWorkflow:
    """Get the global calibration workflow instance."""
    global _workflow
    if _workflow is None:
        _workflow = CalibrationWorkflow()
    return _workflow


def run_calibration(
    calibration_type: CalibrationType, force_redo: bool = False
) -> CalibrationResult:
    """Run a calibration procedure."""
    return get_calibration_workflow().run_calibration(calibration_type, force_redo)


def get_calibration_status() -> Dict[CalibrationType, Dict[str, Any]]:
    """Get status of all calibrations."""
    return get_calibration_workflow().get_calibration_status()


if __name__ == "__main__":
    # Example usage
    workflow = CalibrationWorkflow()

    # Run camera intrinsics calibration
    result = workflow.run_calibration(CalibrationType.CAMERA_INTRINSICS)
    print(f"Camera calibration result: {result.success}")

    # Get status of all calibrations
    status = workflow.get_calibration_status()
    print("Calibration status:")
    for cal_type, info in status.items():
        print(
            f"  {cal_type.value}: {'✅' if info['success'] else '❌'} "
            f"(quality: {info['quality_score']:.2f})"
        )
