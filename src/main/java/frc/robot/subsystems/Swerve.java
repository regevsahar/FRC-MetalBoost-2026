package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.*;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 pigeon;

  // private final SwerveDrivePoseEstimator m_poseEstimator;
  public Swerve() {
    pigeon = new Pigeon2(Constants.SwerveConstants.PigeonID, Constants.CanivoreName);
    zeroPigeon();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    swerveOdometry =
        new SwerveDriveOdometry(
            Constants.SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting
        // pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(8.0, 0.0, 0.0) // Rotation PID constants/ Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );

    // m_poseEstimator =
    //     new SwerveDrivePoseEstimator(
    //         SwerveConstants.swerveKinematics,
    //         pigeon.getRotation2d(),
    //         new SwerveModulePosition[] {
    //         mSwerveMods[0].getPosition(),
    //         mSwerveMods[1].getPosition(),
    //         mSwerveMods[2].getPosition(),
    //         mSwerveMods[3].getPosition()
    //         },
    //         new Pose2d(),
    //         VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    //         VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the speeds of the swerve modules and convert to chassis speeds
    SwerveModuleState[] moduleStates = getModuleStates();

    // Assuming you have logic in place to calculate chassis speeds from module states
    // Here, you would sum the individual module states' speeds to get the total chassis speed
    double vx = 0.0, vy = 0.0, omega = 0.0;

    for (SwerveModuleState state : moduleStates) {
      // You can perform vector math to sum the module states' velocities.
      // This might involve considering the robot's geometry to convert module velocities to chassis
      // speeds.
      vx += state.speedMetersPerSecond * Math.cos(state.angle.getRadians());
      vy += state.speedMetersPerSecond * Math.sin(state.angle.getRadians());
      omega += state.speedMetersPerSecond; // Placeholder for rotational speed (depending on your
      // configuration)
    }

    // Return the combined chassis speeds
    return new ChassisSpeeds(vx, vy, omega);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert the robot-relative speeds into swerve module states
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    // Set the desired state for each swerve module
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(
          swerveModuleStates[mod.moduleNumber], false); // Assuming false for isOpenLoop
    }
  }

  // public void resetTo0 (){
  //     for(SwerveModule mod : mSwerveMods){
  //         mod.setTo0();
  //     }
  // }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void resetPigeon() {
    pigeon.setYaw(0);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveOdometry.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveOdometry.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void zeroPigeon() {
    pigeon.setYaw(0);
  }

  // public void updateOdometry() {
  //     boolean doRejectUpdate = false;

  //     swerveOdometry.update(getGyroYaw(), getModulePositions());

  //     LimelightHelpers.SetRobotOrientation("limelight",
  // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
  //     LimelightHelpers.PoseEstimate mt2 =
  // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  //     if(Math.abs(pigeon.getRate()) > 360) // if our angular velocity is greater than 360 degrees
  // per second, ignore vision updates
  //     {
  //         doRejectUpdate = true;
  //     }
  //     if(mt2.tagCount == 0)
  //     {
  //         doRejectUpdate = true;
  //     }
  //     if(!doRejectUpdate)
  //     {
  //         m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  //         m_poseEstimator.addVisionMeasurement(
  //             mt2.pose,
  //             mt2.timestampSeconds);
  //     }
  // }

  @Override
  public void periodic() {
    // updateOdometry();
    swerveOdometry.update(getGyroYaw(), getModulePositions());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("Pigeon2 Yaw", pigeon.getYaw().getValueAsDouble());
  }
}
