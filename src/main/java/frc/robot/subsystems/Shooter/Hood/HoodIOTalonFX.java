package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor;
  private final CANcoder cancoder;
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, new CANBus(Constants.CanivoreName));
    cancoder = new CANcoder(ShooterConstants.HOOD_CANCODER_ID, new CANBus(Constants.CanivoreName));

    configureTalonFX();
    configureCANCoder();

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

    // Configure CANcoder as feedback sensor
    config.Feedback.FeedbackRemoteSensorID = ShooterConstants.HOOD_CANCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodGearRatio;

    StatusCode status = hoodMotor.getConfigurator().apply(config);
    if (status != StatusCode.OK) {
      System.out.println("Hood TalonFX config failed: " + status);
    }
  }

  private void configureCANCoder() {
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    StatusCode status = cancoder.getConfigurator().apply(cancoderConfig);
    if (status != StatusCode.OK) {
      System.out.println("Failed to configure CANCoder: " + status);
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
    inputs.arc = cancoder.getAbsolutePosition().getValueAsDouble() / ShooterConstants.kHoodRotationsPerDegree;
  }

  @Override
  public void setTargetArc(double arc) {
    // Convert from Arc Degrees to Rotations
    double rotations = arc / ShooterConstants.kHoodRotationsPerDegree;
    hoodMotor.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void stop() {
    hoodMotor.setControl(new DutyCycleOut(0));
  }
}
