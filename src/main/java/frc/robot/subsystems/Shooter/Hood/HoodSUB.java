package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.TunableNumber;

public class HoodSUB extends MBSubsystem {

  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double targetArc = 0.0;

  private final InterpolatingDoubleTreeMap DISTANCE_TO_ARC_MAP = new InterpolatingDoubleTreeMap();

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  public HoodSUB(HoodIO io) {
    super("Hood");
    this.io = io;

    // Mapping (Distance Meters -> Arc)
    DISTANCE_TO_ARC_MAP.put(0.0, 0.0);
    DISTANCE_TO_ARC_MAP.put(0.5, 0.5);
    DISTANCE_TO_ARC_MAP.put(1.0, 1.0);
    DISTANCE_TO_ARC_MAP.put(1.5, 1.5);
    DISTANCE_TO_ARC_MAP.put(2.0, 2.0);
    DISTANCE_TO_ARC_MAP.put(2.5, 2.5);
  }

  public void setTargetArc(double targetArc) {
    this.targetArc = targetArc;
    io.setTargetArc(targetArc);
  }

  public void setTargetDistance() {
    setTargetArc(getArcFromDistance(testDistance.get()));
  }

  public double getArcFromDistance(double distance) {
    return DISTANCE_TO_ARC_MAP.get(distance);
  }

  public double getCurrentArc() {
    return inputs.arc;
  }

  public boolean isAtTarget() {
    return Math.abs(getCurrentArc() - targetArc) < ShooterConstants.kHoodTolerance;
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
