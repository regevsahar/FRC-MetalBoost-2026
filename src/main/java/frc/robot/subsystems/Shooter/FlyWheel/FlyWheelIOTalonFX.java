package frc.robot.subsystems.Shooter.FlyWheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class FlyWheelIOTalonFX implements FlyWheelIO {

  private final TalonFX shooterMotor;
  private final TalonFX slaveMotor;
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);

  public FlyWheelIOTalonFX() {
    shooterMotor =
        new TalonFX(ShooterConstants.MASTER_MOTOR_PORT, new CANBus(Constants.CanivoreName));
    slaveMotor = new TalonFX(ShooterConstants.SLAVE_MOTOR_PORT, new CANBus(Constants.CanivoreName));

    configureTalonFX();

    slaveMotor.setControl(
        new Follower(ShooterConstants.MASTER_MOTOR_PORT, MotorAlignmentValue.Opposed));

    shooterMotor.setNeutralMode(NeutralModeValue.Coast);
    slaveMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  private void configureTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID + Feedforward
    config.Slot0.kP = ShooterConstants.kShooterP.get();
    config.Slot0.kI = ShooterConstants.kShooterI.get();
    config.Slot0.kD = ShooterConstants.kShooterD.get();
    config.Slot0.kS = ShooterConstants.kShooterS.get();
    config.Slot0.kV = ShooterConstants.kShooterV.get();
    config.Slot0.kA = ShooterConstants.kShooterA.get();

    StatusCode status = shooterMotor.getConfigurator().apply(config);
    if (status != StatusCode.OK) {
      System.out.println("Shooter TalonFX config failed: " + status);
    }

    slaveMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (ShooterConstants.kShooterP.hasChanged()
        || ShooterConstants.kShooterI.hasChanged()
        || ShooterConstants.kShooterD.hasChanged()
        || ShooterConstants.kShooterS.hasChanged()
        || ShooterConstants.kShooterV.hasChanged()
        || ShooterConstants.kShooterA.hasChanged()) {
      configureTalonFX();
    }

    inputs.rpm = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  @Override
  public void setTargetRPM(double targetRPM) {
    shooterMotor.setControl(velocityControl.withVelocity(targetRPM / 60.0));
  }

  @Override
  public void stop() {
    shooterMotor.setControl(new DutyCycleOut(0));
  }
}
