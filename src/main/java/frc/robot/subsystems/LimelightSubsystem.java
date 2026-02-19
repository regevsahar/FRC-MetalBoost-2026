package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.lib.util.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  private final String limelightName;

  public LimelightSubsystem(String limelightName) {
    this.limelightName = limelightName;
  }

  /** מחזיר את ה-Pose לפי MegaTag (botpose_wpiblue/red) בהתאם לצבע האליינס. */
  public Optional<LimelightHelpers.PoseEstimate> getMegaTag2Pose() {
    LimelightHelpers.PoseEstimate estimate;

    estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    if (estimate == null || estimate.pose == null)
      return Optional.empty();
    return Optional.of(estimate);
  }

  public Optional<LimelightHelpers.PoseEstimate> getMegaTag1Pose() {
    LimelightHelpers.PoseEstimate estimate;

    estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

    if (estimate == null || estimate.pose == null)
      return Optional.empty();
    return Optional.of(estimate);
  }

  public void SetHeading(Rotation2d heading) {
    LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0, 0, 0, 0, 0);
  }

  public void setHeading(Rotation2d heading) {
    LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0, 0, 0, 0, 0);
  }

  /** latency של ה-Limelight בשניות */
  public double getLatency() {
    Optional<Alliance> alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
    LimelightHelpers.PoseEstimate estimate;

    if (alliance.get() == Alliance.Red) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    } else {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    } else {
      estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    if (estimate == null)
      return 0.0;

    double now = Timer.getFPGATimestamp();
    return now - estimate.timestampSeconds;
  }

  /**
   * האם יש target
   */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(limelightName);
  }

  public String getName() {
    return limelightName;
  }

  public double getApriltagID() {
    return LimelightHelpers.getFiducialID(limelightName);
  }

  public double getDistanceFromTarget() {
    return LimelightHelpers.getTargetPose_CameraSpace(limelightName)[0];
  }

  @Override
  public void periodic() {
    Optional<LimelightHelpers.PoseEstimate> mt1 = getMegaTag1Pose();
    Optional<LimelightHelpers.PoseEstimate> mt2 = getMegaTag2Pose();

    Logger.recordOutput("Limelight-DATA/" + getName() + "/HasTarget", hasTarget());
    Logger.recordOutput("Limelight-DATA/" + getName() + "/PrimaryTagID", hasTarget() ? getApriltagID() : -1);
    Logger.recordOutput("Limelight-DATA/" + getName() + "/MT1-isPresent", mt1.isPresent());
    Logger.recordOutput("Limelight-DATA/" + getName() + "/MT2-isPresent", mt2.isPresent());
    Logger.recordOutput("Limelight-DATA/" + getName() + "/Pose-isPresent",
        mt2.isPresent() && mt2.get().pose != null);
    Logger.recordOutput("Limelight-DATA/" + getName() + "/DistanceFromTarget",
        hasTarget() ? getDistanceFromTarget() : -1);
    Logger.recordOutput("Limelight-DATA/" + getName() + "/MT1", getMegaTag1Pose().get().pose);
    Logger.recordOutput("Limelight-DATA/" + getName() + "/MT2", getMegaTag2Pose().get().pose);

  }
}
