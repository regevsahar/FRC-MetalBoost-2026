package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import frc.lib.util.LimelightHelpers; // <- המחלקת עזר שהאתר נותן

public class LimelightSubsystem extends SubsystemBase {

    private final String limelightName;

    public LimelightSubsystem(String limelightName) {
        this.limelightName = limelightName;
    }

    /**
     * מחזיר את ה-Pose לפי MegaTag (botpose_wpiblue/red)
     * בהתאם לצבע האליינס.
     */
    public Optional<LimelightHelpers.PoseEstimate> getMegaTag2Pose() {
        LimelightHelpers.PoseEstimate estimate;
        
        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        

        if (estimate == null || estimate.pose == null) return Optional.empty();
        return Optional.of(estimate);
    }
    public Optional<LimelightHelpers.PoseEstimate> getMegaTag1Pose() {
        LimelightHelpers.PoseEstimate estimate;
        
        estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        

        if (estimate == null || estimate.pose == null) return Optional.empty();
        return Optional.of(estimate);
    }
    
    public void SetHeading(Rotation2d heading){
        LimelightHelpers.SetRobotOrientation(limelightName, heading.getDegrees(), 0, 0, 0, 0, 0);
    }
    /**
     * latency של ה-Limelight בשניות
     */
    public double getLatency() {
        Optional<Alliance> alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        LimelightHelpers.PoseEstimate estimate;

        if (alliance.get() == Alliance.Red) {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
        } else {
            estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }

        if (estimate == null) return 0.0;

        double now = Timer.getFPGATimestamp();
        return now - estimate.timestampSeconds;
    }
    /**
     * האם יש target
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }
}
