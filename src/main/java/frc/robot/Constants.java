package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.Swerve.COTSTalonFXSwerveConstants;
import frc.lib.util.Swerve.SwerveModuleConstants;

public final class Constants {
  public static final double stickDeadband = 0.1;

  public static final int LedId = 9;
  public static final int LedLength = 150;

  public static final String CanivoreName = "CanivoreMB";

  public static final double ManualHoodStrength = 0.25;

  public static final class SwerveConstants {
    public static final boolean invertGyro = false; // TODO: Always ensure Gyro is CCW+ CW-

    public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4N.KrakenX60(
        COTSTalonFXSwerveConstants.SDS.MK4N.driveRatios.L3);

    public static final int PigeonID = 30;

    /* Drivetrain Constants */
    public static final double trackWidth = 0.56;
    public static final double wheelBase = 0.56;
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // 8 //TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
    public static final double driveKV = 1.51;
    public static final double driveKA = 0.27;

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot

    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

    /** Meters per Second per Second */
    public static final double maxAcceleration = 2.8; // TODO: This must be tuned to specific robot

    /** Radians per Second per Second */
    public static final double maxAngularAcceleration = 8; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake; // coast
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* PathPlanner constraints */
    public static final PathConstraints constraints = new PathConstraints(
        LinearVelocity.ofBaseUnits(Constants.SwerveConstants.maxSpeed, MetersPerSecond),
        LinearAcceleration.ofBaseUnits(
            Constants.SwerveConstants.maxAcceleration, MetersPerSecondPerSecond),
        AngularVelocity.ofBaseUnits(
            Constants.SwerveConstants.maxAngularVelocity, RadiansPerSecond),
        AngularAcceleration.ofBaseUnits(
            Constants.SwerveConstants.maxAngularAcceleration, RadiansPerSecondPerSecond));

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 19;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 0;
      public static final int canCoderID = 34;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
    // tuned to specific robot
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
