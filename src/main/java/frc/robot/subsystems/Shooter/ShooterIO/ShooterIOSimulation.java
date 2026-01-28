package frc.robot.subsystems.Shooter.ShooterIO;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class ShooterIOSimulation implements ShooterIO {

  private final FlywheelSim sim;
  private final PIDController pid;
  private final SimpleMotorFeedforward ff;
  private double appliedVolts = 0.0;
  private double targetRPM = 0.0;

  public ShooterIOSimulation() {
    // Create the linear system for the flywheel
    LinearSystem<N1, N1, N1> plant =
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX60(2),
            ShooterConstants.kMomentOfInertia,
            ShooterConstants.kGearRatio);

    // Initialize FlywheelSim with the plant and motor
    sim = new FlywheelSim(plant, DCMotor.getKrakenX60(2));

    // Initialize Control
    pid = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    ff = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Simulating the on-board controller (e.g., TalonFX)
    if (targetRPM != 0) {
      appliedVolts =
          pid.calculate(sim.getAngularVelocityRPM(), targetRPM) + ff.calculate(targetRPM);
      appliedVolts = Math.max(-16.0, Math.min(16.0, appliedVolts));
    }

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02); // Standard loop time of 20ms

    inputs.rpm = sim.getAngularVelocityRPM();
    inputs.distance = inputs.distance + (getSurfaceSpeedMetersPerSecond(inputs.rpm) * 0.02);
  }

  @Override
  public void setTargetRPM(double targetRPM) {
    this.targetRPM = targetRPM;
  }

  @Override
  public void stop() {
    targetRPM = 0.0;
    appliedVolts = 0.0;
    sim.setInputVoltage(0.0);
  }

  /** Converts RPM to surface speed (Meters per Second) */
  public double getSurfaceSpeedMetersPerSecond(double rpm) {
    double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    return radiansPerSecond * ShooterConstants.kWheelRadius;
  }

  /** Converts Surface Speed (Meters per Second) to RPM */
  public double getRpmFromSurfaceSpeed(double metersPerSecond) {
    double radiansPerSecond = metersPerSecond / ShooterConstants.kWheelRadius;
    return Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond);
  }
}
