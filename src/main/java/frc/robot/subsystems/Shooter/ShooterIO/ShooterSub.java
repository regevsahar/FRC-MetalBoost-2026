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
    DISTANCE_TO_RPM_MAP.put(0.0, 0.0);
    DISTANCE_TO_RPM_MAP.put(1.0, 2000.0);
    DISTANCE_TO_RPM_MAP.put(2.0, 3000.0);
    DISTANCE_TO_RPM_MAP.put(3.0, 3500.0);
    DISTANCE_TO_RPM_MAP.put(4.0, 4000.0);
    DISTANCE_TO_RPM_MAP.put(5.0, 4500.0);
    DISTANCE_TO_RPM_MAP.put(6.0, 5000.0);
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
