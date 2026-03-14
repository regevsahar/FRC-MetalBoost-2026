package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Vision.PoseEstimator;

public class HoodSUB extends MBSubsystem {

  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double targetArc = 0.0;
  private PoseEstimator poseEstimator;

  // Tunables
  public HoodSUB(HoodIO io, PoseEstimator poseEstimator) {
    super("Hood");
    this.poseEstimator = poseEstimator;
    this.io = io;
  }

  public boolean isStalling() {
    if (RobotBase.isSimulation()) {
      return false;
    }
    HoodIOTalonFX hood = (HoodIOTalonFX) io;
    return hood.isStalling();
  }

  public void resetPosition() {
    io.resetPosition();
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void setTargetArc(double targetArc) {
    this.targetArc = targetArc;
    io.setTargetArc(targetArc);
  }

  public void setTargetDistanceFromHub() {
    double distance = poseEstimator.getDistanceFromHub();
    setTargetArc(getArcFromDistance(distance));
  }

  public void setTargetDistance(double distance) {
    setTargetArc(getArcFromDistance(distance));
  }

  public double getArcFromDistance(double distance) {
    return ShooterConstants.DISTANCE_TO_ARC_MAP.get(distance);
  }

  public double getCurrentArc() {
    return inputs.arc;
  }

  public boolean isAtTarget() {
    return Math.abs(getCurrentArc() - targetArc) < ShooterConstants.kArcTolerance;
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void subsystemPeriodic() {

    io.updateInputs(inputs);

    SmartDashboard.putNumber("/Shooter/Arc/Current", inputs.arc);

    SmartDashboard.putNumber("/Shooter/Arc/Target", targetArc);

    SmartDashboard.putBoolean("/Shooter/Arc/At Target", isAtTarget());
  }
}
