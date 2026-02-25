package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class IntakeSubsystem extends MBSubsystem {
  /** Creates a new IntakeSubsystem. */
  private TalonFX motor;

  public IntakeSubsystem() {
    super("IntakeSubsystem");
    motor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID, new CANBus(Constants.CanivoreName));
  }

  public void setMotor(double value) {
    motor.set(value);
  }

}
