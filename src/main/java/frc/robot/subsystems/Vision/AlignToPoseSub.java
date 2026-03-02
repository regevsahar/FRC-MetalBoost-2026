package frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Vision.VisionConstants.AlignToPoseConstants;
import org.littletonrobotics.junction.Logger;

public class AlignToPoseSub extends MBSubsystem {

  private final PIDController pidController;

  public AlignToPoseSub() {
    super("AlignToPose");
    pidController =
        new PIDController(
            VisionConstants.AlignToPoseConstants.kP,
            VisionConstants.AlignToPoseConstants.kI,
            VisionConstants.AlignToPoseConstants.kD);
    pidController.setTolerance(VisionConstants.AlignToPoseConstants.kToleranceRad);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double calculateRotationOutput(Pose2d currentPose, Translation2d targetPose) {
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();

    double angleToTarget = Math.atan2(dy, dx);

    double omega = pidController.calculate(currentPose.getRotation().getRadians(), angleToTarget);

    omega =
        clamp(
            omega,
            -AlignToPoseConstants.kMaxOmegaRadPerSec,
            AlignToPoseConstants.kMaxOmegaRadPerSec);

    return omega;
  }

  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public void resetToCurrent(Pose2d currentPose) {
    pidController.reset();
    pidController.setSetpoint(currentPose.getRotation().getRadians());
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  @Override
  public void subsystemPeriodic() {
    Logger.recordOutput("AlignToPoseSubsystem/isAtSetpoint", atSetpoint());
    // This method will be called once per scheduler run
  }
}
