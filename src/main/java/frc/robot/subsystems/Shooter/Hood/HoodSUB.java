package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.MBSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodSUB extends MBSubsystem {

  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();
  private double targetArc = 0.0;

  // Tunables
  private final TunableNumber testDistance = new TunableNumber("Shooter/Distance", 0.0);

  // SysId
  private final SysIdRoutine sysIdRoutine;

  public HoodSUB(HoodIO io) {
    super("Hood");
    this.io = io;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Units.Volts.of(1).per(Units.Second), // ramp rate: 1 V/s
                Units.Volts.of(7), // step voltage: 7 V
                null, // default timeout
                null), // default state handler
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Units.Volts)),
                null, // no log consumer needed; use Tuner X / URCL externally
                this,
                "Hood"));
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

  // ── SysId ────────────────────────────────────────────────────────────────

  /**
   * Quasistatic (slow-ramp) SysId command.
   *
   * @param direction {@link SysIdRoutine.Direction#kForward} or {@link
   *     SysIdRoutine.Direction#kReverse}
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Dynamic (step-voltage) SysId command.
   *
   * @param direction {@link SysIdRoutine.Direction#kForward} or {@link
   *     SysIdRoutine.Direction#kReverse}
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  @Override
  public void subsystemPeriodic() {

    io.updateInputs(inputs);

    SmartDashboard.putNumber("/Shooter/Arc/Current", inputs.arc);

    SmartDashboard.putNumber("/Shooter/Arc/Target", targetArc);

    SmartDashboard.putBoolean("/Shooter/Arc/At Target", isAtTarget());
  }
}
