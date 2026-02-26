package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor;

  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final PositionVoltage positionControl = new PositionVoltage(0);

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, new CANBus(Constants.CanivoreName));

    resetPosition();
    configMotor();

    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private double degreesToRotations(double deg) {
    return (deg - 30) / 44.8;
  }

  private double rotationsToDegrees(double rot) {
    return rot * 44.8 + 30;
  }

  private void configMotor() {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = ShooterConstants.kHoodP.get();
    config.Slot0.kI = ShooterConstants.kHoodI.get();
    config.Slot0.kD = ShooterConstants.kHoodD.get();
    config.Slot0.kS = ShooterConstants.kHoodS.get();
    config.Slot0.kV = ShooterConstants.kHoodV.get();
    config.Slot0.kA = ShooterConstants.kHoodA.get();

    // SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();

    // softLimits.ForwardSoftLimitEnable = true;
    // softLimits.ReverseSoftLimitEnable = true;

    // softLimits.ForwardSoftLimitThreshold =
    // degreesToRotations(ShooterConstants.kMaxArc);

    // softLimits.ReverseSoftLimitThreshold =
    // degreesToRotations(ShooterConstants.kMinArc);

    // config.SoftwareLimitSwitch = softLimits;

    for (int i = 0; i < 5; i++) {
      if (hoodMotor.getConfigurator().apply(config).isOK()) {
        return;
      }
    }
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    inputs.arc = rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());

    if (ShooterConstants.kHoodP.hasChanged()
        || ShooterConstants.kHoodI.hasChanged()
        || ShooterConstants.kHoodD.hasChanged()
        || ShooterConstants.kHoodS.hasChanged()
        || ShooterConstants.kHoodV.hasChanged()
        || ShooterConstants.kHoodA.hasChanged()) {
      configMotor();
    }
  }

  public double getCurrentArc() {
    return rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setTargetArc(double arcDegrees) {
    hoodMotor.setControl(positionControl.withPosition(degreesToRotations(arcDegrees)));
  }

  public void resetPosition() {
    hoodMotor.setPosition(0);
  }

  @Override
  public void stop() {
    hoodMotor.setControl(dutyCycleControl.withOutput(0));
  }
}
