package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.enums.Reef;
import frc.lib.util.PathPlannerUtil;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Swerve;

public class GoToNearestBranch extends Command {

  private final PoseEstimator estimator;
  private final Swerve swerve;
  private Pose2d current;
  private Pose2d target;

  public GoToNearestBranch(PoseEstimator estimator, Swerve swerve) {
    this.estimator = estimator;
    this.swerve = swerve;

    addRequirements(estimator, swerve);
  }

  public static Reef getClosestBranch(Pose2d robotPose) {
    Reef closest = null;
    double minDistance = Double.MAX_VALUE;

    for (Reef branch : Reef.values()) {
      double distance = branch.getDistance(robotPose);
      if (distance < minDistance) {
        minDistance = distance;
        closest = branch;
      }
    }

    return closest;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    current = estimator.getEstimatedPosition();
    target = getClosestBranch(current).getPose();

    // TODO check if mirror is needed
    PathPlannerUtil.createPathDuringRuntime(
        current, target, Constants.SwerveConstants.constraints, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PathPlannerUtil.isRobotInPathfindingDeadband(current, target);
  }
}
