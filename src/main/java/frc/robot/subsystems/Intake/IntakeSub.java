package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference; // simulation
import com.ctre.phoenix6.sim.TalonFXSimState; // simulation
import edu.wpi.first.math.system.plant.DCMotor; // simulation
import edu.wpi.first.wpilibj.RobotController; // simulation
import edu.wpi.first.wpilibj.simulation.ElevatorSim; // simulation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class IntakeSub extends MBSubsystem {

  private TalonFX motor;
  private double target = 0;

  private final ElevatorSim elevator;
  private final TalonFXSimState simState;

  public IntakeSub() {
    super("Intake");

    elevator =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            IntakeConstants.kGearRatio,
            IntakeConstants.kCarriageMassKg,
            IntakeConstants.kDrumRadiusMeters,
            IntakeConstants.kMinHeightMeters,
            IntakeConstants.kMaxHeightMeters,
            false, // simulateGravity
            IntakeConstants.kMinHeightMeters); // starting height

    motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, new CANBus(Constants.CanivoreName));
    simState = motor.getSimState();
    simState.setMotorType(TalonFXSimState.MotorType.KrakenX60);

    configureTalonFX();
    motor.setNeutralMode(NeutralModeValue.Brake);
    simState.Orientation = ChassisReference.Clockwise_Positive;
    resetPosition();
  }

  private void configureTalonFX() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

    // feedback configuration
    talonFXConfig.Slot0.kP = IntakeConstants.kP.get();
    talonFXConfig.Slot0.kI = IntakeConstants.kI.get();
    talonFXConfig.Slot0.kD = IntakeConstants.kD.get();
    talonFXConfig.Slot0.kS = IntakeConstants.kS.get();
    talonFXConfig.Slot0.kV = IntakeConstants.kV.get();
    talonFXConfig.Slot0.kA = IntakeConstants.kA.get();
    motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.kAcceleration.get();
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeConstants.kCruiseVelocity.get();
    motionMagicConfigs.MotionMagicJerk = IntakeConstants.kJerk.get();

    talonFXConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kGearRatio;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // apply the configuration
    StatusCode status = motor.getConfigurator().apply(talonFXConfig);
    if (status != StatusCode.OK) {
      System.out.println("Failed to configure TalonFX: " + status);
    }
  }

  public void setTargetPosition(double target) {
    this.target = target;
    motor.setControl(new PositionVoltage(target));
  }

  public void resetPosition() {
    motor.setPosition(0);
  }

  public void stop() {
    motor.setControl(new DutyCycleOut(0));
  }

  public void SetMotor(double value) {
    motor.setVoltage(value);
  }

  public double getCurrentPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public boolean isStalling() {
    return Math.abs(motor.getStatorCurrent().getValueAsDouble()) > IntakeConstants.kStallThreshold;
  }

  public boolean isAtTargetPosition(double targetPosition) {
    double error = Math.abs(getCurrentPosition() - targetPosition);
    return error < IntakeConstants.kTolerance;
  }

  @Override
  public void subsystemPeriodic() {
    SmartDashboard.putNumber("/Intake/Current Position", getCurrentPosition());
    SmartDashboard.putNumber("/Intake/Target Position", target);
    SmartDashboard.putBoolean("/Intake/At Target", isAtTargetPosition(getCurrentPosition()));
    SmartDashboard.putNumber("/Intake/position", motor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("/Intake/Stator Current", motor.getStatorCurrent().getValueAsDouble());

    if (IntakeConstants.kP.hasChanged()
        || IntakeConstants.kI.hasChanged()
        || IntakeConstants.kD.hasChanged()
        || IntakeConstants.kS.hasChanged()
        || IntakeConstants.kV.hasChanged()
        || IntakeConstants.kA.hasChanged()
        || IntakeConstants.kAcceleration.hasChanged()
        || IntakeConstants.kCruiseVelocity.hasChanged()
        || IntakeConstants.kJerk.hasChanged()) {
      configureTalonFX();
    }
  }

  @Override
  public void simulationPeriodic() {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    double ctrlVolts = simState.getMotorVoltage(); // what the controller asks for
    elevator.setInputVoltage(simState.getMotorVoltage());
    elevator.update(0.020);

    double mechRot =
        elevator.getPositionMeters() / (2 * Math.PI * IntakeConstants.kDrumRadiusMeters);
    simState.setRawRotorPosition(mechRot * IntakeConstants.kGearRatio);
    simState.setRotorVelocity(
        elevator.getVelocityMetersPerSecond()
            / (2 * Math.PI * IntakeConstants.kDrumRadiusMeters)
            * IntakeConstants.kGearRatio);

    System.out.println(
        "target="
            + target
            + " ctrlVolts="
            + ctrlVolts
            + " meters="
            + elevator.getPositionMeters()
            + " mechRot="
            + mechRot
            + " reported="
            + motor.getPosition().getValueAsDouble()
            + " mode="
            + motor.getControlMode(true)
            + " slot="
            + motor.getClosedLoopSlot(true).getValue()
            + " clErr="
            + motor.getClosedLoopError(true).getValueAsDouble()
            + " clOut="
            + motor.getClosedLoopOutput(true).getValueAsDouble()
            + " pOut="
            + motor.getClosedLoopProportionalOutput(true).getValueAsDouble()
            + " ff="
            + motor.getClosedLoopFeedForward(true).getValueAsDouble()
            + " cmd="
            + getCurrentCommand());
  }
}
