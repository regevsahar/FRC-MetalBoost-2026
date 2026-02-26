package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MBSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodSub extends MBSubsystem {

  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double targetArc = 0.0;

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  public HoodSub(HoodIO io) {
    super("Hood");
    this.io = io;

  }

  public void setTargetArc(double targetArc) {
    this.targetArc = targetArc;
    io.setTargetArc(targetArc);
  }

  public void setTargetDistance() {
    setTargetArc(getArcFromDistance(testDistance.get()));
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
