// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;

public class IntakeRollersSub extends MBSubsystem {

    private TalonFX motor;

    public IntakeRollersSub() {
        super("Intake rollers");
        motor = new TalonFX(IntakeConstants.INTAKE_ROLLER_PORT, new CANBus(Constants.CanivoreName));
        configureTalonFX();
    }

    private void configureTalonFX() {
        TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

        StatusCode statusInsert = motor.getConfigurator().apply(talonFXConfig);
        if (statusInsert != StatusCode.OK) {
            System.out.println("Failed to configure TalonFX: " + statusInsert);
        }
    }

    public void SetSpeed(double value) {
        motor.set(value);
    }

    public void StopIntake() {
        motor.set(0);
    }

    @Override
    public void subsystemPeriodic() {
    }
}