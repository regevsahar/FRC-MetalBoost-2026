package frc.robot.subsystems.Shooter.FlyWheel;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.util.TunableNumber;

public class FlyWheelSub extends MBSubsystem {

  private final FlyWheelIO io;
  private final FlyWheelIO.FlyWheelIOInputs inputs = new FlyWheelIO.FlyWheelIOInputs();
  private double targetRPM = 0.0;

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  // SysId
  private final SysIdRoutine sysIdRoutine;

  public FlyWheelSub(FlyWheelIO io) {
    super("Shooter");
    this.io = io;

    sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.of(1).per(Units.Second), // ramp rate: 1 V/s
            Units.Volts.of(3), // step voltage: 10 V (flywheel needs authority)
            null,
            null),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage.in(Units.Volts)), null, this, "Flywheel"));
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

  // ── SysId ────────────────────────────────────────────────────────────────

  /**
   * Quasistatic (slow-ramp) SysId command.
   *
   * @param direction {@link SysIdRoutine.Direction#kForward} or {@link
   *                  SysIdRoutine.Direction#kReverse}
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Dynamic (step-voltage) SysId command.
   *
   * @param direction {@link SysIdRoutine.Direction#kForward} or {@link
   *                  SysIdRoutine.Direction#kReverse}
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void subsystemPeriodic() {

    io.updateInputs(inputs);

    SmartDashboard.putNumber("/Shooter/RPM/Current", inputs.rpm);

    SmartDashboard.putNumber("/Shooter/RPM/Target", targetRPM);

    SmartDashboard.putBoolean("/Shooter/RPM/At Target", isAtTarget());
  }
}
