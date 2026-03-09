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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class IntakeSub extends MBSubsystem {

  private TalonFX motor;

  public IntakeSub() {
    super("Intake");
    motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT, new CANBus(Constants.CanivoreName));

    configureTalonFX();
    motor.setNeutralMode(NeutralModeValue.Brake);
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
    // limit configuration
    // double forwardLimitRotations = IntakeConstants.BOTTOM_LIMIT;
    // double reverseLimitRotations = IntakeConstants.TOP_LIMIT;

    // var softLimit = talonFXConfig.SoftwareLimitSwitch;
    // softLimit.ForwardSoftLimitEnable = true;
    // softLimit.ForwardSoftLimitThreshold = forwardLimitRotations;
    // softLimit.ReverseSoftLimitEnable = true;
    // softLimit.ReverseSoftLimitThreshold = reverseLimitRotations;

    // apply the configuration
    StatusCode status = motor.getConfigurator().apply(talonFXConfig);
    if (status != StatusCode.OK) {
      System.out.println("Failed to configure TalonFX: " + status);
    }
  }

  public void setTargetPosition(double target) {
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
    return motor.getStatorCurrent().getValueAsDouble() > IntakeConstants.kStallThreshold;
  }

  public boolean isAtTargetPosition(double targetPosition) {
    double error = Math.abs(getCurrentPosition() - targetPosition);
    return error < IntakeConstants.kTolerance;
  }

  @Override
  public void subsystemPeriodic() {
    SmartDashboard.putNumber("/Intake/Current Position", getCurrentPosition());
    SmartDashboard.putNumber("/Intake/Target Position", getCurrentPosition());
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
}
