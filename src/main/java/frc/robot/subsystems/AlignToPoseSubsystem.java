package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlignToPoseSubsystem extends SubsystemBase {

  private final PIDController pidController;

  public AlignToPoseSubsystem() {
    pidController = new PIDController(
        Constants.AlignToPoseConstants.kP,
        Constants.AlignToPoseConstants.kI,
        Constants.AlignToPoseConstants.kD);
    pidController.setTolerance(Constants.AlignToPoseConstants.kToleranceRad);
    pidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double calculateRotationOutput(Pose2d currentPose, Translation2d targetPose) {
    double dx = targetPose.getX() - currentPose.getX();
    double dy = targetPose.getY() - currentPose.getY();

    double angleToTarget = Math.atan2(dy, dx);

    double omega = pidController.calculate(currentPose.getRotation().getRadians(), angleToTarget);

    omega = clamp(
        omega,
        -Constants.AlignToPoseConstants.kMaxOmegaRadPerSec,
        Constants.AlignToPoseConstants.kMaxOmegaRadPerSec);

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
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
