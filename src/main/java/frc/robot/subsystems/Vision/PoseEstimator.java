package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldUtils.FieldPoses;
import frc.lib.util.Vision.LimelightHelpers;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends MBSubsystem {
  public SwerveDrivePoseEstimator sEstimator;
  public TimeInterpolatableBuffer<Double> turretYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
  public TimeInterpolatableBuffer<Rotation2d> gyroYawBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
  public Pose2d visionPose = new Pose2d();
  Field2d field = new Field2d();
  private double offsetX = 0;
  private double offsetY = 0;
  private int nOffsets = 0;
  private Translation2d hubTarget = FieldPoses.HUB_CENTER_BLUE;

  public PoseEstimator() {
    super("PoseEstimator");
    sEstimator = new SwerveDrivePoseEstimator(
        Constants.SwerveConstants.swerveKinematics,
        new Rotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        new Pose2d(),
        VisionConstants.PoseEstimator.stateStdDevs,
        VisionConstants.PoseEstimator.visionStdDevs);
    SmartDashboard.putData("FieldPoseEstimator", field);
    hubTarget = FieldPoses.getHubPosByAliiance();
  }

  public void updateHeadingOffset(Rotation2d gyro, Rotation2d vision) {
    // double dx = Math.cos(vision.getRadians() - gyro.getRadians());
    double dx = vision.minus(gyro).getCos();
    // double dy = Math.sin(vision.getRadians() - gyro.getRadians());
    double dy = vision.minus(gyro).getSin();

    offsetX = (offsetX * nOffsets + dx) / (nOffsets + 1);
    offsetY = (offsetY * nOffsets + dy) / (nOffsets + 1);
    nOffsets++;
  }

  public Rotation2d getCorrectedHeading(Rotation2d gyro) {
    if (nOffsets == 0)
      return gyro;
    return new Rotation2d(Math.atan2(offsetY, offsetX)).plus(gyro);
  }

  /**
   * Check if this returns true before using {@link #updateVision()}
   *
   * @return If time buffers are !null
   */
  public boolean readyToUpdateVision() {
    return gyroYawBuffer.getSample(0).isPresent();
  }

  /**
   * Update estimator with Swerve States and Gyro Yaw data. Needs to be updated
   * every loop.
   */
  // public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[]
  // modulePositions){
  // sEstimator.update(gyroAngle, modulePositions);
  // gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
  // }

  public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    sEstimator.update(getCorrectedHeading(gyroAngle), modulePositions);
    gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle);
  }

  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
    sEstimator.resetPosition(getCorrectedHeading(gyroAngle), modulePositions, newPose);
  }

  /**
   * Update estimator with vision data. Should only be updated when target is
   * visible.
   *
   * @param LLlatency seconds
   */
  // public void updateVision(Pose2d LLpose, double LLlatency){
  // double timeStamp = Timer.getFPGATimestamp() - LLlatency;
  // Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
  // sEstimator.addVisionMeasurement(
  // new Pose2d(LLpose.getX(), LLpose.getY(), gyro),
  // timeStamp
  // );
  // }
  public Optional<Rotation2d> getGyroYawAtTimeStamp(double timestamp) {
    return gyroYawBuffer.getSample(timestamp);
  }

  public void updateVision(
      LimelightHelpers.PoseEstimate estimate, Rotation2d currentGyro, double distanceFromTag) {
    double timestamp = estimate.timestampSeconds;
    double translationSTDev = Math.max(
        Math.pow(distanceFromTag, 2) * VisionConstants.PoseEstimator.stdDevFactorTranslation,
        VisionConstants.PoseEstimator.minimumStdDev);
    double rotationSTDev = Math.max(
        Math.pow(distanceFromTag, 2) * VisionConstants.PoseEstimator.stdDevFactorRotation,
        VisionConstants.PoseEstimator.minimumStdDev);
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(translationSTDev, translationSTDev, rotationSTDev);
    sEstimator.addVisionMeasurement(estimate.pose, timestamp, visionStdDevs);
  }

  public double getDistanceFromHub() {
    return getEstimatedPosition().getTranslation().getDistance(hubTarget);
  }

  public double getRoundedDistanceFromHub() {
    return Math.round(getDistanceFromHub() * 10.0) / 10.0;
  }

  public Pose2d getEstimatedPosition() {
    return sEstimator.getEstimatedPosition();
  }

  private final List<LimelightSub> limelights = new ArrayList<>();

  public void addLimelight(LimelightSub limelight) {
    limelights.add(limelight);
  }

  @Override
  public void subsystemPeriodic() {
    SmartDashboard.putNumber("robotX", getEstimatedPosition().getX());
    SmartDashboard.putNumber("robotY", getEstimatedPosition().getY());
    SmartDashboard.putNumber("robotHeading", getEstimatedPosition().getRotation().getRadians());

    Logger.recordOutput("Estimator/DistanceFromHub", getDistanceFromHub());
    Logger.recordOutput("Estimator/RoundedDistanceFromHub", getRoundedDistanceFromHub());
    Logger.recordOutput("Estimator/CurrHub", this.hubTarget);
    Logger.recordOutput("Estimator/estimator", getEstimatedPosition());
    Logger.recordOutput("Estimator/Pose2d/robotX", getEstimatedPosition().getX());
    Logger.recordOutput("Estimator/Pose2d/robotY", getEstimatedPosition().getY());
    Logger.recordOutput(
        "Estimator/Pose2d/robotHeading", getEstimatedPosition().getRotation().getRadians());

    Optional<Rotation2d> currentGyroOpt = gyroYawBuffer.getSample(Timer.getFPGATimestamp());
    if (currentGyroOpt.isPresent()) {
      Rotation2d currentGyro = currentGyroOpt.get();
      Rotation2d correctedHeading = getCorrectedHeading(currentGyro);

      for (LimelightSub limelight : limelights) {
        limelight.setHeading(correctedHeading);

        Optional<LimelightHelpers.PoseEstimate> mt1 = limelight.getMegaTag1Pose();
        Optional<LimelightHelpers.PoseEstimate> mt2 = limelight.getMegaTag2Pose();

        if (mt1.isPresent() && mt2.isPresent()) {
          Optional<Rotation2d> gyroYawAtTimeStamp = getGyroYawAtTimeStamp(mt2.get().timestampSeconds);
          if (limelight.hasTarget() && gyroYawAtTimeStamp.isPresent()) {
            double distanceFromTag = limelight.getDistanceFromTarget();

            updateHeadingOffset(gyroYawAtTimeStamp.get(), mt1.get().pose.getRotation());
            updateVision(mt2.get(), gyroYawAtTimeStamp.get(), distanceFromTag);
          }
        }
      }
    }

    field.setRobotPose(getEstimatedPosition());
  }
}
