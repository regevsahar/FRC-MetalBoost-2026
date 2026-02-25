// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SpinDexser;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpindexerSubsystem extends SubsystemBase {
  /** Creates a new Spindexer. */
  private TalonFX shooterTriggerMotor;

  private SparkMax motor;

  public SpindexerSubsystem() {
    motor = new SparkMax(59, MotorType.kBrushless);
    shooterTriggerMotor = new TalonFX(8, new CANBus(Constants.CanivoreName));
  }

  public void SetMotor(double value) {
    motor.set(value);
    shooterTriggerMotor.set(-value);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
