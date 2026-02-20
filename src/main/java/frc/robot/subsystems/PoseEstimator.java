package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.util.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class PoseEstimator extends SubsystemBase{
    public SwerveDrivePoseEstimator sEstimator;
    public TimeInterpolatableBuffer<Double> turretYawBuffer = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    public TimeInterpolatableBuffer<Rotation2d> gyroYawBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
    public Pose2d visionPose = new Pose2d();
    Field2d field = new Field2d();
    //buffer לזווית של הגירו
    private double offsetX = 0;
    private double offsetY = 0;
    private int nOffsets = 0;
    private Translation2d target = Constants.FieldConstants.HUB_CENTER_BLUE;

    public PoseEstimator(){
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
            Constants.PoseEstimator.stateStdDevs, 
            Constants.PoseEstimator.visionStdDevs
        );
        var alliance = DriverStation.getAlliance();
        SmartDashboard.putData("FieldPoseEstimator", field);
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            target = Constants.FieldConstants.HUB_CENTER_RED;
        }

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
    if (nOffsets == 0) return gyro;
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

  /** Update estimator with Swerve States and Gyro Yaw data. Needs to be updated every loop. */
  // public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[]
  // modulePositions){
  // sEstimator.update(gyroAngle, modulePositions);
  // gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle.getRadians());
  // }

  public void updateSwerve(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    sEstimator.update(getCorrectedHeading(gyroAngle), modulePositions);
    gyroYawBuffer.addSample(Timer.getFPGATimestamp(), gyroAngle);
  }

    /** Update estimator with vision data. 
     *  Should only be updated when target is visible.
     * @param LLlatency seconds */
    // public void updateVision(Pose2d LLpose, double LLlatency){
    //     double timeStamp = Timer.getFPGATimestamp() - LLlatency;
    //     Rotation2d gyro = new Rotation2d(gyroYawBuffer.getSample(timeStamp).get());
    //     sEstimator.addVisionMeasurement(
    //         new Pose2d(LLpose.getX(), LLpose.getY(), gyro),
    //         timeStamp
    //     );
    // }
    public Optional<Rotation2d> getGyroYawAtTimeStamp(double timestamp){
        return gyroYawBuffer.getSample(timestamp);
    }
    public void updateVision(LimelightHelpers.PoseEstimate estimate,Rotation2d currentGyro,double distanceFromTag) {
        double timestamp = estimate.timestampSeconds;  
        double translationSTDev = Math.max(Math.pow(distanceFromTag, 2) * Constants.PoseEstimator.stdDevFactorTranslation, Constants.PoseEstimator.minimumStdDev);
        double rotationSTDev = Math.max(Math.pow(distanceFromTag, 2) * Constants.PoseEstimator.stdDevFactorRotation, Constants.PoseEstimator.minimumStdDev);
        Matrix<N3,N1> visionStdDevs = VecBuilder.fill(translationSTDev, translationSTDev, rotationSTDev);
        sEstimator.addVisionMeasurement(
            estimate.pose,
            timestamp,
            visionStdDevs
        );
    }

    public double getDistanceFromHub(){
        return getEstimatedPosition().getTranslation().getDistance(target);
    }

  public Pose2d getEstimatedPosition() {
    return sEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("robotX", getEstimatedPosition().getX());
    SmartDashboard.putNumber("robotY", getEstimatedPosition().getY());
    SmartDashboard.putNumber("robotHeading", getEstimatedPosition().getRotation().getRadians());

        Logger.recordOutput("Estimator/DistanceFromHub", getDistanceFromHub());
        Logger.recordOutput("Estimator/estimator", getEstimatedPosition());
        Logger.recordOutput("Estimator/Pose2d/robotX", getEstimatedPosition().getX());
        Logger.recordOutput("Estimator/Pose2d/robotY", getEstimatedPosition().getY());
        Logger.recordOutput("Estimator/Pose2d/robotHeading",getEstimatedPosition().getRotation().getRadians());


    field.setRobotPose(getEstimatedPosition());
  }
}
