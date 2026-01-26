// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;

public class Shooter extends MBSubsystem {
  /** Creates a new Shooter. */
  private TalonFX masterMotor, slaveMotor;

  int masterMotorId = 9;

  public Shooter() {
    super("Shooter");
    masterMotor = new TalonFX(masterMotorId, new CANBus(Constants.CanivoreName));
    slaveMotor = new TalonFX(13, new CANBus(Constants.CanivoreName));
    slaveMotor.setControl(new Follower(masterMotorId, MotorAlignmentValue.Opposed));
  }

  public void SetMotor(double value) {
    masterMotor.set(value);
  }

  @Override
  public void subsystemPeriodic() {
    // This method will be called once per scheduler run
  }
}
