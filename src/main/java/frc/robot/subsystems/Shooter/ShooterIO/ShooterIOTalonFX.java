package frc.robot.subsystems.Shooter.ShooterIO;

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

public class ShooterIOTalonFX implements ShooterIO {

    private final TalonFX shooterMotor;
    private final TalonFX slaveMotor;
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public ShooterIOTalonFX() {
        shooterMotor = new TalonFX(ShooterConstants.MASTER_MOTOR_PORT, Constants.CanivoreName);
        slaveMotor = new TalonFX(ShooterConstants.SLAVE_MOTOR_PORT, Constants.CanivoreName);

        configureTalonFX();

        // Follows the master motor
        slaveMotor.setControl(new Follower(ShooterConstants.MASTER_MOTOR_PORT, MotorAlignmentValue.Opposed));
        
        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        slaveMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    private void configureTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID + Feedforward
        config.Slot0.kP = ShooterConstants.kShooterP;
        config.Slot0.kI = ShooterConstants.kShooterI;
        config.Slot0.kD = ShooterConstants.kShooterD;
        config.Slot0.kS = ShooterConstants.kShooterS;
        config.Slot0.kV = ShooterConstants.kShooterV;
        config.Slot0.kA = ShooterConstants.kShooterA;

        StatusCode status = shooterMotor.getConfigurator().apply(config);
        if (status != StatusCode.OK) {
            System.out.println("Shooter TalonFX config failed: " + status);
        }
        
        // Apply same config to slave to ensure compatible settings if needed, 
        // though follower handles control. Good practice to configure limits etc.
        slaveMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // getVelocity returns rotations per second, convert to RPM
        inputs.rpm = shooterMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    @Override
    public void setTargetRPM(double targetRPM) {
        // VelocityVoltage expects rotations per second
        shooterMotor.setControl(velocityControl.withVelocity(targetRPM / 60.0));
    }

    @Override
    public void stop() {
        shooterMotor.setControl(new DutyCycleOut(0));
    }
}