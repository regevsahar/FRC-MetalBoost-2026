package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOSim implements HoodIO {

  private final SingleJointedArmSim sim;
  private final PIDController pid;
  private final ArmFeedforward ff;
  private double appliedVolts = 0.0;
  private double targetArc = 0.0;

  public HoodIOSim() {
    sim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.kHoodGearRatio,
            0.005,
            0.18,
            Units.degreesToRadians(ShooterConstants.kMinArc),
            Units.degreesToRadians(ShooterConstants.kMaxArc),
            true,
            Units.degreesToRadians(ShooterConstants.kMinArc));

    pid =
        new PIDController(
            ShooterConstants.kHoodP.get(),
            ShooterConstants.kHoodI.get(),
            ShooterConstants.kHoodD.get());
    ff =
        new ArmFeedforward(
            ShooterConstants.kHoodS.get(),
            ShooterConstants.kHoodV.get(),
            ShooterConstants.kHoodA.get());
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (ShooterConstants.kHoodP.hasChanged()
        || ShooterConstants.kHoodI.hasChanged()
        || ShooterConstants.kHoodD.hasChanged()) {
      pid.setPID(
          ShooterConstants.kHoodP.get(),
          ShooterConstants.kHoodI.get(),
          ShooterConstants.kHoodD.get());
    }

    double currentAngleRadians = sim.getAngleRads();
    double targetAngleRadians = Units.degreesToRadians(targetArc);

    appliedVolts =
        pid.calculate(currentAngleRadians, targetAngleRadians)
            + ff.calculate(currentAngleRadians, 0); // Position control ff
    appliedVolts =
        Math.max(
            -ShooterConstants.kMaxVoltage, Math.min(ShooterConstants.kMaxVoltage, appliedVolts));

    sim.setInputVoltage(appliedVolts);
    sim.update(ShooterConstants.kLoopTime);

    inputs.arc = Units.radiansToDegrees(sim.getAngleRads());
  }

  @Override
  public void setTargetArc(double arc) {
    this.targetArc = arc;
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }
}
