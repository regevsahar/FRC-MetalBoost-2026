// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyance;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class ConveyanceSub extends MBSubsystem {
  
 private TalonFX lowerMotor,upperMotor;

  public ConveyanceSub() {
    super("Conveyance wheels");
    lowerMotor = new TalonFX(ConveyanceConstants.LOWER_MOTOR_PORT,new CANBus(Constants.CanivoreName));
    upperMotor = new TalonFX(ConveyanceConstants.UPPER_MOTOR_PORT, new CANBus(Constants.CanivoreName));
    upperMotor.setControl(new Follower(ConveyanceConstants.LOWER_MOTOR_PORT, MotorAlignmentValue.Opposed));
  }


    public void setSpeed(double value){
      lowerMotor.set(value);
    }

    public void stop(){
      lowerMotor.set(0);
      upperMotor.set(0);
    }
  
  @Override
  public void subsystemPeriodic() {
    // This method will be called once per scheduler run
  }
}