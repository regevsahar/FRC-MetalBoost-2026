// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Conveyance;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class ConveyanceSub extends MBSubsystem {

  private TalonFX lowerMotor, upperMotor;
  private ColorSensorV3 colorSensor;

  public ConveyanceSub() {
    super("Conveyance wheels");
    lowerMotor =
        new TalonFX(ConveyanceConstants.LOWER_MOTOR_PORT, new CANBus(Constants.CanivoreName));
    upperMotor =
        new TalonFX(ConveyanceConstants.UPPER_MOTOR_PORT, new CANBus(Constants.CanivoreName));
    upperMotor.setControl(
        new Follower(ConveyanceConstants.LOWER_MOTOR_PORT, MotorAlignmentValue.Opposed));
    colorSensor = new ColorSensorV3(ConveyanceConstants.COLOR_SENSOR_PORT);
  }

  public void setSpeed(double value) {
    lowerMotor.setVoltage(value);
  }

  public void stop() {
    lowerMotor.set(0);
  }

  public boolean hasBall() {
    return colorSensor.getProximity() > ConveyanceConstants.BALL_DETECTION_THRESHOLD;
  }

  @Override
  public void subsystemPeriodic() {
    SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
  }
}
