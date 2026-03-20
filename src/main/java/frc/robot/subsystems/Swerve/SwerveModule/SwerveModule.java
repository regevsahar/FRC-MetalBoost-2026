package frc.robot.subsystems.Swerve.SwerveModule;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.Swerve.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;

  private TalonFX mAngleMotor;
  private TalonFX mDriveMotor;
  private CANcoder angleEncoder;

  private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
      Constants.SwerveConstants.driveKS,
      Constants.SwerveConstants.driveKV,
      Constants.SwerveConstants.driveKA);

  /* drive motor control requests */
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final VoltageOut driveVoltage = new VoltageOut(0);

  /* angle motor control requests */
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, new CANBus(Constants.CanivoreName));
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID, new CANBus(Constants.CanivoreName));
    mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    resetToAbsolute();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID, new CANBus(Constants.CanivoreName));
    TalonFXConfiguration driveConfig = Robot.ctreConfigs.swerveDriveFXConfig;

    mDriveMotor.getConfigurator().apply(driveConfig);

    mDriveMotor.getConfigurator().setPosition(0.0);
  }

  private SwerveModuleState targetState = new SwerveModuleState();

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    desiredState.optimize(getState().angle);
    this.targetState = desiredState;

    mAngleMotor.setControl(anglePosition.withPosition((desiredState.angle.getRotations())));
    // -(desiredState.angle.getRotations()) TODO check
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      mDriveMotor.setControl(driveDutyCycle);
    } else {
      driveVelocity.Velocity = Conversions.MPSToRPS(
          desiredState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference);
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      mDriveMotor.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void resetToAbsolute() {
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.RPSToMPS(
            mDriveMotor.getVelocity().getValueAsDouble(),
            Constants.SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            mDriveMotor.getPosition().getValueAsDouble(),
            Constants.SwerveConstants.wheelCircumference),
        Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble()));
  }

  public void setTo0() {
    mAngleMotor.setPosition(0);
  }

  // -------------------------------------------------------------------------
  // SysId helpers
  // -------------------------------------------------------------------------

  /**
   * Forces a steering angle without optimization. Used exclusively during SysId.
   */
  public void setAngle(Rotation2d angle) {
    mAngleMotor.setControl(anglePosition.withPosition(angle.getRotations()));
  }

  /**
   * Forces a steering angle without optimization. Used exclusively during SysId.
   */
  public void setAngle(Rotation2d angle) {
    mAngleMotor.setControl(anglePosition.withPosition(angle.getRotations()));
  }

  /**
   * Commands a raw voltage to the drive motor. Used exclusively during SysId characterization — do
   * NOT call during normal teleop/auto.
   */
  public void setDriveVoltage(double volts) {
    mDriveMotor.setControl(driveVoltage.withOutput(volts));
  }

  /** Drive wheel position in meters (for SysId position log). */
  public double getDrivePositionMeters() {
    return Conversions.rotationsToMeters(
        mDriveMotor.getPosition().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference);
  }

  /** Drive wheel velocity in m/s (for SysId velocity log). */
  public double getDriveVelocityMPS() {
    return Conversions.RPSToMPS(
        mDriveMotor.getVelocity().getValueAsDouble(), Constants.SwerveConstants.wheelCircumference);
  }
}
