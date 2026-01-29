package frc.robot.subsystems.Shooter.ShooterIO;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.TunableNumber;

public class ShooterSub extends MBSubsystem {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();
  private double targetRPM = 0.0;

  private final InterpolatingDoubleTreeMap DISTANCE_TO_RPM_MAP = new InterpolatingDoubleTreeMap();

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  public ShooterSub(ShooterIO io) {
    super("Shooter");
    this.io = io;

    // Mapping (Distance Meters -> RPM)
    DISTANCE_TO_RPM_MAP.put(0.0, 2600.0);
    DISTANCE_TO_RPM_MAP.put(0.5, 2675.0);
    DISTANCE_TO_RPM_MAP.put(1.0, 2800.0);
    DISTANCE_TO_RPM_MAP.put(1.5, 3000.0);
    DISTANCE_TO_RPM_MAP.put(2.0, 3250.0);
    DISTANCE_TO_RPM_MAP.put(2.5, 3400.0);
  }

  public void setTargetRPM(double targetRPM) {
    this.targetRPM = targetRPM;
    io.setTargetRPM(targetRPM);
  }

  public void setTargetDistance() {
    setTargetRPM(getRPMFromDistance(testDistance.get()));
  }

  public double getRPMFromDistance(double distance) {
    return DISTANCE_TO_RPM_MAP.get(distance);
  }

  public double getCurrentRPM() {
    return inputs.rpm;
  }

  public boolean isAtTarget() {
    return Math.abs(getCurrentRPM() - targetRPM) < ShooterConstants.kRPMPolerance;
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void subsystemPeriodic() {

    io.updateInputs(inputs);

    SmartDashboard.putNumber("/Shooter/RPM/Current", inputs.rpm);

    SmartDashboard.putNumber("/Shooter/RPM/Target", targetRPM);

    SmartDashboard.putBoolean("/Shooter/RPM/At Target", isAtTarget());
  }
}
