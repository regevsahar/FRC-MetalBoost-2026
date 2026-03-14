// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyance;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class ConveyanceRollerSub extends MBSubsystem {

  private TalonFX motor;

  public ConveyanceRollerSub() {
    super("Conveyance rollers");
    motor =
        new TalonFX(ConveyanceConstants.ROLLERS_PORT_ROLLERS, new CANBus(Constants.CanivoreName));
    configureTalonFX();
  }

  private void configureTalonFX() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    StatusCode statusInsert = motor.getConfigurator().apply(talonFXConfig);
    if (statusInsert != StatusCode.OK) {
      System.out.println("Failed to configure TalonFX: " + statusInsert);
    }
  }

  public void setSpeed(double value) {
    motor.setVoltage(value);
  }

  public void stop() {
    motor.set(0);
  }

  @Override
  public void subsystemPeriodic() {
    // This method will be called once per scheduler run
  }
}
