package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOTalonFX implements HoodIO {

  private final TalonFX hoodMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

  // Status Signals for optimization
  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> motorVoltageSignal;
  private final StatusSignal<Double> supplyCurrentSignal;
  private final StatusSignal<Double> tempSignal;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID, new CANBus(Constants.CanivoreName));

    // Configure the motor
    configMotor();

    // Initialize Signals
    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    motorVoltageSignal = hoodMotor.getMotorVoltage();
    supplyCurrentSignal = hoodMotor.getSupplyCurrent();
    tempSignal = hoodMotor.getDeviceTemp();

    // Optimize bus utilization
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionSignal,
        velocitySignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        tempSignal);

    // Disable or lower unused signals if necessary (optional, but
    // setUpdateFrequencyForAll handles the main ones)
    hoodMotor.optimizeBusUtilization();

    // Set initial neutral mode
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  private void configMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // PID Configs
    config.Slot0.kP = ShooterConstants.kHoodP.get();
    config.Slot0.kI = ShooterConstants.kHoodI.get();
    config.Slot0.kD = ShooterConstants.kHoodD.get();
    config.Slot0.kS = ShooterConstants.kHoodS.get();
    config.Slot0.kV = ShooterConstants.kHoodV.get();
    config.Slot0.kA = ShooterConstants.kHoodA.get();

    // Current Limits: 20A continuous, 40A peak
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = 20.0;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentThreshold = 40.0;
    currentLimits.SupplyTimeThreshold = 0.1;
    config.CurrentLimits = currentLimits;

    // Soft Limits (Prevent mechanism damage)
    SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;
    // Convert degrees to rotations
    softLimits.ForwardSoftLimitThreshold = ShooterConstants.kMaxArc * ShooterConstants.kHoodRotationsPerDegree;
    softLimits.ReverseSoftLimitThreshold = ShooterConstants.kMinArc * ShooterConstants.kHoodRotationsPerDegree;
    config.SoftwareLimitSwitch = softLimits;

    // Retry configuration application
    for (int i = 0; i < 5; i++) {
      if (hoodMotor.getConfigurator().apply(config).isOK()) {
        return;
      }
    }
    System.out.println("Hood TalonFX config failed after 5 attempts!");
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Refresh signals
    BaseStatusSignal.refreshForAll(
        positionSignal,
        velocitySignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        tempSignal);

    // Populate inputs
    inputs.arc = positionSignal.getValueAsDouble() / ShooterConstants.kHoodRotationsPerDegree;
    inputs.velocity = velocitySignal.getValueAsDouble() / ShooterConstants.kHoodRotationsPerDegree;
    inputs.appliedVoltage = motorVoltageSignal.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrentSignal.getValueAsDouble();
    inputs.tempCelcius = tempSignal.getValueAsDouble();

    // Check for tunable updates
    if (ShooterConstants.kHoodP.hasChanged()
        || ShooterConstants.kHoodI.hasChanged()
        || ShooterConstants.kHoodD.hasChanged()
        || ShooterConstants.kHoodS.hasChanged()
        || ShooterConstants.kHoodV.hasChanged()
        || ShooterConstants.kHoodA.hasChanged()) {
      configMotor();
    }
  }

  @Override
  public void setTargetArc(double arc) {
    // Clamp target to soft limits for extra safety
    double clampedArc = Math.max(ShooterConstants.kMinArc, Math.min(ShooterConstants.kMaxArc, arc));

    // Convert from Arc Degrees to Rotations
    double rotations = clampedArc * ShooterConstants.kHoodRotationsPerDegree;

    // Check if within tolerance? PID handles it so just command it.
    hoodMotor.setControl(positionControl.withPosition(rotations));
  }

  @Override
  public void stop() {
    hoodMotor.setControl(dutyCycleControl.withOutput(0));
  }
}
