package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose3d;

public interface LoggableMechanism3d {
  /**
   * Returns the 3D displayable object's position.
   *
   * @param parentPose3d the pose of the parent object in the 3D scene (if applicable)
   * @return the 3D displayable object's position
   */
  Pose3d getDisplayPose3d(Pose3d parentPose3d);
}
