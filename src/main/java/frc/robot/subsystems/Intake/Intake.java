package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class Intake extends MBSubsystem {

  private TalonFX motor;

  public Intake() {
    super("Intake");
    motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, new CANBus(Constants.CanivoreName));

    configureTalonFX();
    motor.setNeutralMode(NeutralModeValue.Brake);
    resetPosition();
  }

  private void configureTalonFX() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    // feedback configuration
    talonFXConfig.Slot0.kP = IntakeConstants.kP;
    talonFXConfig.Slot0.kI = IntakeConstants.kI;
    talonFXConfig.Slot0.kD = IntakeConstants.kD;
    talonFXConfig.Slot0.kS = IntakeConstants.kS;
    talonFXConfig.Slot0.kV = IntakeConstants.kV;
    talonFXConfig.Slot0.kA = IntakeConstants.kA;
    talonFXConfig.Feedback.SensorToMechanismRatio = IntakeConstants.kGearRatio;

    // limit configuration
    // Soft limits are in Rotations. Convert cm to Rotations.
    double forwardLimitRotations = cmToRotations(IntakeConstants.BOTTOM_LIMIT);
    double reverseLimitRotations = cmToRotations(IntakeConstants.TOP_LIMIT);

    var softLimit = talonFXConfig.SoftwareLimitSwitch;
    softLimit.ForwardSoftLimitEnable = true;
    softLimit.ForwardSoftLimitThreshold = forwardLimitRotations;
    softLimit.ReverseSoftLimitEnable = true;
    softLimit.ReverseSoftLimitThreshold = reverseLimitRotations;

    // apply the configuration
    StatusCode status = motor.getConfigurator().apply(talonFXConfig);
    if (status != StatusCode.OK) {
      System.out.println("Failed to configure TalonFX: " + status);
    }
  }

  public void setTargetPosition(double targetCm) {
    motor.setControl(new PositionVoltage(cmToRotations(targetCm)));
  }

  public void resetPosition() {
    motor.setPosition(0);
  }

  public void StopMotor() {
    motor.setControl(new DutyCycleOut(0));
  }

  public void SetMotor(double value) {
    motor.set(value);
  }

  public double getCurrentPosition() {
    return rotationsToCm(motor.getPosition().getValueAsDouble());
  }

  public double rotationsToCm(double rotations) {
    return rotations / IntakeConstants.ROTATIONS_PER_CM;
  }

  public double cmToRotations(double cm) {
    return cm * IntakeConstants.ROTATIONS_PER_CM;
  }

  public boolean isAtTargetPosition(double targetPositionCm) {
    double error = Math.abs(getCurrentPosition() - targetPositionCm);
    return error < IntakeConstants.kTolerance;
  }

  @Override
  public void subsystemPeriodic() {
    SmartDashboard.putNumber("/Intake/Current Position", getCurrentPosition());
  }
}
