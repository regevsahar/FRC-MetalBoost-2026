package frc.robot.subsystems.Shooter.FlyWheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class FlyWheelSub extends MBSubsystem {

  private final FlyWheelIO io;
  private final FlyWheelIO.FlyWheelIOInputs inputs = new FlyWheelIO.FlyWheelIOInputs();
  private double targetRPM = 0.0;

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  public FlyWheelSub(FlyWheelIO io) {
    super("Shooter");
    this.io = io;
  }

  public void setTargetRPM(double targetRPM) {
    this.targetRPM = targetRPM;
    io.setTargetRPM(targetRPM);
  }

  public void setTargetDistance() {
    setTargetRPM(getRPMFromDistance(testDistance.get()));
  }

  public double getRPMFromDistance(double distance) {
    return ShooterConstants.DISTANCE_TO_RPM_MAP.get(distance);
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
