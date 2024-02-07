from commands2 import Subsystem, Command
from wpimath.geometry import Transform3d, Transform2d, Pose2d, Pose3d
from wpimath.units import feetToMeters
from photonlibpy.photonCamera import (
    PhotonCamera,
    setVersionCheckEnabled,
    PhotonPipelineResult,
)
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from robotpy_apriltag import AprilTagFieldLayout
from wpilib import RobotBase
from typing import List, Dict

import os


class VisionSystem(Subsystem):
    """
    Class to manage the various cameras and their targeting on the Robot
    """

    def __init__(self) -> None:
        super().__init__()

        # Value to track whether or not we've gotten good information from the camera
        self._timeout_in_seconds = 1

        if RobotBase.isReal():
            self._pcamera = AprilTagPhotonCamera("test", Pose3d())

    def periodic(self) -> None:
        return super().periodic()


class CameraPoseEstimate:
    def __init__(self, obs_time: float, pose_estimate: Pose2d) -> None:
        self._timestamp: float = obs_time
        self._estimate: Pose2d = Pose2d

class TagMetaData:
    def __init__(self, id: int, yaw: float):
        self._id = id
        self._yaw = yaw


class AprilTagPhotonCamera:
    def __init__(self, name: str, center_offset: Transform3d) -> None:
        setVersionCheckEnabled(False)

        self._camera: PhotonCamera = PhotonCamera(name)
        self._center_offset: Transform3d = center_offset

        self._pose_estimates: List = []

        self._tag_metadatas: Dict = {}

        tagPath = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "deploy", "2024-crescendo.json")
        )
        self._fieldTagLayout: AprilTagFieldLayout = AprilTagFieldLayout(tagPath)

    def update_camera_results(self, previousPoseEstimate: Pose2d) -> None:
        # Grab the latest results from the camera. A result suggests that a camera frame
        # was processed, not that it found something.  We need to filter on the results
        # to get the best pose estimate
        result: PhotonPipelineResult = self._camera.getLatestResult()
        result_time: float = result.getTimestamp()

        # Update the dashboard to let drivers know we're functional

        # Clear the old values
        self._pose_estimates.clear()
        self._tag_metadatas.clear()

        # Process each PhotonTrackedTarget
        # Use this as a basis for target selection:
        # https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html#d-to-3d-ambiguity
        # Use your odometry history from all sensors to pick the pose closest to where you expect the robot to be.
        # Reject poses which are very unlikely (ex: outside the field perimeter, or up in the air)
        # Ignore pose estimates which are very close together (and hard to differentiate)
        # Use multiple cameras to look at the same target, such that at least one camera can generate a good pose estimate
        # Look at many targets at once, using each to generate multiple pose estimates. Discard the outlying estimates, use the ones which are tightly clustered together.
        for target in result.getTargets():
            target_ID = target.getFiducialId()

            # Make sure that we've gotten an actual april tag, should be numbered
            if target_ID >= 0:
                fieldPose: Pose3d = self._fieldTagLayout.getTagPose(target_ID)
                if fieldPose is not None:
                    candidates = []
                    # Add the primary result to the list first, then add alternates
                    candidates.append(
                        self.__target_to_field_Pose(
                            fieldPose, target.getBestCameraToTarget()
                        )
                    )
                    candidates.append(
                        self.__target_to_field_Pose(
                            fieldPose, target.getAlternateCameraToTarget()
                        )
                    )

                    self._tag_metadatas[target_ID] = TagMetaData(target_ID, target.getYaw)

                    # Now with a candidate list, filter for the best results
                    filtered_candidates: List[Pose2d] = []
                    for candidate in candidates:
                        if self.__pose_on_field(candidate):
                            filtered_candidates.append(candidate)

                        # Add any other filtering here in the future

                    # Start with nothing, and replace if we find something
                    best_candidate = None
                    best_candidate_distance = 9999999
                    for candidate in filtered_candidates:
                        diff = (candidate - previousPoseEstimate).translation().norm()

                        if diff < best_candidate_distance:
                            best_candidate_distance = diff
                            best_candidate = candidate

                    # If we found something worth storing, store it
                    if best_candidate is not None:
                        self._pose_estimates.append(
                            CameraPoseEstimate(result_time, best_candidate)
                        )

    def get_pose_estimates(self) -> List:
        return self._pose_estimates

    def __target_to_field_Pose(self, targetPose: Pose3d, offset: Transform3d) -> Pose2d:
        cameraPose = targetPose.transformBy(self._center_offset.inverse())
        return cameraPose.transformBy(self._center_offset.inverse()).toPose2d()

    def __pose_on_field(self, pose: Pose2d) -> bool:
        translation: Transform2d = pose.translation()

        # Get the x,y values from the translation
        x = translation.X()
        y = translation.Y()

        # Translate field dimensions to meters, to match the Translation values
        # Then compare to make sure the location is within the field boundaries
        x_in_field: bool = 0.0 <= x <= feetToMeters(54)
        y_in_field: bool = 0.0 <= y <= feetToMeters(27)

        return x_in_field and y_in_field
    

class NoteDetectionPhotonCamera:
    def __init__(self) -> None:

