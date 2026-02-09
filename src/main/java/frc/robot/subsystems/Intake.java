package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX motor;

  public Intake() {
    motor = new TalonFX(14, new CANBus(Constants.CanivoreName));
  }

  public void setMotor(double value) {
    motor.set(value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
