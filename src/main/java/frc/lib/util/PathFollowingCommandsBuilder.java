package frc.lib.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.function.Supplier;

public class PathFollowingCommandsBuilder {

  public static Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
    return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
  }

  public static Command pathfindToPose(
      Pose2d targetPose,
      PathConstraints pathfindingConstraints,
      double goalEndVelocityMetersPerSecond) {
    return AutoBuilder.pathfindToPose(
        targetPose, pathfindingConstraints, goalEndVelocityMetersPerSecond);
  }

  public static Command pathfindThenFollowPath(
      PathPlannerPath path, PathConstraints pathfindingConstraints) {
    return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
  }

  public static Command pathfindThenFollowPath(
      PathPlannerPath path,
      PathConstraints pathfindingConstraints,
      double velocityBetweenPathfindingToPathFollowingMetersPerSecond) {
    return AutoBuilder.pathfindToPose(
            Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path)),
            pathfindingConstraints,
            velocityBetweenPathfindingToPathFollowingMetersPerSecond)
        .andThen(followPath(path));
  }

  public static Command followPathOrPathfindAndFollowPath(
      PathPlannerPath path, Supplier<Pose2d> currentPose, PathConstraints pathfindingConstraints) {
    return new ConditionalCommand(
        followPath(path),
        pathfindThenFollowPath(path, pathfindingConstraints),
        () ->
            PathPlannerUtil.isRobotInPathfindingDeadband(
                currentPose.get(),
                Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path))));
  }
}
