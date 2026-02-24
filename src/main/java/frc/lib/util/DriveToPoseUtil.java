package frc.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseUtil {

  /** Creates command that makes the robot move to certain poses (X,Y,Rotation) */
  public Command driveToPoseHolonomic(
      Pose2d currentPose, double maxVel, double maxAccel, Pose2d... targetPoses) {
    if (targetPoses == null || targetPoses.length == 0) {
      throw new IllegalArgumentException("At least one target pose is required");
    }

    PathConstraints constraints = new PathConstraints(maxVel, maxAccel, 10, 8);

    Logger.recordOutput("Estimator/Pose2d/currentPose", currentPose);
    Logger.recordOutput("Estimator/Pose2d/targetPoses", targetPoses);

    // Combine current pose + all targets into one array
    Pose2d[] allPoses = new Pose2d[targetPoses.length + 1];
    allPoses[0] = currentPose;
    System.arraycopy(targetPoses, 0, allPoses, 1, targetPoses.length);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(allPoses);

    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, targetPoses[targetPoses.length - 1].getRotation()));

    return AutoBuilder.followPath(path);
  }

  public Command driveToPoseHolonomic(
      Pose2d currentPose, Pose2d targetPose, double maxVel, double maxAccel) {
    // Call supplier to get live position
    // Constraints creation
    PathConstraints constraints = new PathConstraints(maxVel, maxAccel, 10, 8);

    Logger.recordOutput("Estimator/Pose2d/targetPose", targetPose);
    Logger.recordOutput("Estimator/Pose2d/currentPose", currentPose);
    // Waypoint creation
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);

    // PathPlannerPath creation
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(
                0.0, // velocity at end (0 = stop)
                targetPose.getRotation() // End rotation
                ));

    return AutoBuilder.followPath(path);
  }
}
