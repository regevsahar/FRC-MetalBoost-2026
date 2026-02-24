package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShotPrediction {

  /**
   * Predicts the robot's future position based on current pose, field-relative speeds, and flight
   * time. futurePos = currentPos + (vx, vy) * flightTime
   */
  public static Translation2d predictFuturePosition(
      Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, double flightTime) {
    double futureX = currentPose.getX() + fieldRelativeSpeeds.vxMetersPerSecond * flightTime;
    double futureY = currentPose.getY() + fieldRelativeSpeeds.vyMetersPerSecond * flightTime;
    return new Translation2d(futureX, futureY);
  }

  /**
   * Calculates the desired robot yaw to face the hub from the predicted future position. desiredYaw
   * = atan2(hubY - futureY, hubX - futureX)
   */
  public static Rotation2d calculateDesiredYaw(
      Translation2d futurePosition, Translation2d hubPosition) {
    double dx = hubPosition.getX() - futurePosition.getX();
    double dy = hubPosition.getY() - futurePosition.getY();
    return new Rotation2d(Math.atan2(dy, dx));
  }
}
