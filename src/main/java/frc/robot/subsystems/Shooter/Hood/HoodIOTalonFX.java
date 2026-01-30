package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {

    private final TalonFX hoodMotor;
    private final PositionVoltage positionControl = new PositionVoltage(0);

    public HoodIOTalonFX() {
        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, new CANBus(Constants.CanivoreName));

        configureTalonFX();

        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configureTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.kHoodP.get();
        config.Slot0.kI = ShooterConstants.kHoodI.get();
        config.Slot0.kD = ShooterConstants.kHoodD.get();
        config.Slot0.kS = ShooterConstants.kHoodS.get();
        config.Slot0.kV = ShooterConstants.kHoodV.get();
        config.Slot0.kA = ShooterConstants.kHoodA.get();

        // Set current limits if needed, for now sticking to defaults or add later

        StatusCode status = hoodMotor.getConfigurator().apply(config);
        if (status != StatusCode.OK) {
            System.out.println("Hood TalonFX config failed: " + status);
        }
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Check for tunable updates
        if (ShooterConstants.kHoodP.hasChanged()
                || ShooterConstants.kHoodI.hasChanged()
                || ShooterConstants.kHoodD.hasChanged()
                || ShooterConstants.kHoodS.hasChanged()
                || ShooterConstants.kHoodV.hasChanged()
                || ShooterConstants.kHoodA.hasChanged()) {
            configureTalonFX();
        }

        // Convert from Rotations to Arc Degrees
        inputs.arc = hoodMotor.getPosition().getValueAsDouble() / ShooterConstants.kHoodRotationsPerDegree;
    }

    @Override
    public void setTargetArc(double arc) {
        // Convert from Arc Degrees to Rotations
        double rotations = arc * ShooterConstants.kHoodRotationsPerDegree;
        hoodMotor.setControl(positionControl.withPosition(rotations));
    }

    @Override
    public void stop() {
        hoodMotor.setControl(new DutyCycleOut(0));
    }
}
