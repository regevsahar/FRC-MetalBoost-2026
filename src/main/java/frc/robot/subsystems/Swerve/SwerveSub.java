package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.MBSubsystem;
import frc.robot.subsystems.Swerve.SwerveModule.SwerveModule;
import frc.robot.subsystems.Vision.PoseEstimator;
import org.littletonrobotics.junction.Logger;

public class SwerveSub extends MBSubsystem {

  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public Pigeon2 pigeon;
  private Command currentCommand;
  private Field2d field = new Field2d();
  RobotConfig config = null;
  private Command currentPathCommand;
  private PoseEstimator estimator;
  private CANBus m_canBus;

  // ---------------------------------------------------------------------------
  // SysId Routine — characterizes the drive motors (kS, kV, kA, kP)
  // ---------------------------------------------------------------------------
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Units.Volts.per(Units.Second).of(0.5), // ramp rate
              Units.Volts.of(2.0), // step voltage
              null, // default timeout
              (state) -> SignalLogger.writeString("SysIdTestState", state.toString())),
          new SysIdRoutine.Mechanism(
              // Drive: send the same voltage to every module's drive motor
              // while locking steering to 0° (straight ahead)
              (voltage) -> {
                for (SwerveModule mod : mSwerveMods) {
                  // Lock steer to 0 rotations (straight forward)
                  mod.setAngle(Rotation2d.fromDegrees(0));
                  mod.setDriveVoltage(voltage.in(Volts));
                }
              },
              // Log: average position and velocity across all four modules
              // The SysIdRoutine framework captures the applied voltage automatically.
              (log) -> {
                double avgPositionMeters = 0;
                double avgVelocityMPS = 0;
                for (SwerveModule mod : mSwerveMods) {
                  avgPositionMeters += mod.getDrivePositionMeters();
                  avgVelocityMPS += mod.getDriveVelocityMPS();
                }
                avgPositionMeters /= 4.0;
                avgVelocityMPS /= 4.0;

                log.motor("swerve-drive")
                    .linearPosition(Meters.of(avgPositionMeters))
                    .linearVelocity(MetersPerSecond.of(avgVelocityMPS));
              },
              this));

  // private final SwerveDrivePoseEstimator m_poseEstimator;
  public SwerveSub(PoseEstimator estimator) {
    super("Swerve");
    this.estimator = estimator;
    pigeon = new Pigeon2(Constants.SwerveConstants.PigeonID, new CANBus(Constants.CanivoreName));
    m_canBus = new CANBus(Constants.CanivoreName);
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

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.5, 0.0, 0.0), new PIDConstants(7.5, 0.0, 0.0)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // m_poseEstimator =
    // new SwerveDrivePoseEstimator(
    // SwerveConstants.swerveKinematics,
    // pigeon.getRotation2d(),
    // new SwerveModulePosition[] {
    // mSwerveMods[0].getPosition(),
    // mSwerveMods[1].getPosition(),
    // mSwerveMods[2].getPosition(),
    // mSwerveMods[3].getPosition()
    // },
    // new Pose2d(),
    // VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", field);
  }

  // ---------------------------------------------------------------------------
  // SysId command factories
  // Call these from RobotContainer and bind to buttons run in Test mode.
  // ---------------------------------------------------------------------------

  /** Slowly ramp voltage forward — use to characterize kS and kV. */
  public Command sysIdQuasistaticForward() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  /** Slowly ramp voltage backward. */
  public Command sysIdQuasistaticReverse() {
    return m_sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  /** Step to full voltage forward — use to characterize kA. */
  public Command sysIdDynamicForward() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
  }

  /** Step to full voltage backward. */
  public Command sysIdDynamicReverse() {
    return m_sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command followPath(PathPlannerPath path) {
    currentPathCommand = AutoBuilder.followPath(path);
    return currentPathCommand;
  }

  public boolean isPathFinished() {
    return currentPathCommand != null && currentPathCommand.isFinished();
  }

  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    estimator.resetPose(getGyroYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the speeds of the swerve modules and convert to chassis speeds
    SwerveModuleState[] moduleStates = getModuleStates();

    // Assuming you have logic in place to calculate chassis speeds from module
    // states
    // Here, you would sum the individual module states' speeds to get the total
    // chassis speed
    double vx = 0.0, vy = 0.0, omega = 0.0;

    for (SwerveModuleState state : moduleStates) {
      // You can perform vector math to sum the module states' velocities.
      // This might involve considering the robot's geometry to convert module
      // velocities to chassis speeds.
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
          swerveModuleStates[mod.moduleNumber], true); // Assuming false for isOpenLoop
    }
  }

  // public void resetTo0 (){
  // for(SwerveModule mod : mSwerveMods){
  // mod.setTo0();
  // }
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

  public SwerveModuleState[] getTargetModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getTargetState();
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
    return estimator.getEstimatedPosition();
    // return swerveOdometry.getPoseMeters();
  }

  public Pose2d getOdometryPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    estimator.resetPose(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    Pose2d newPose = new Pose2d(getPose().getTranslation(), heading);
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), newPose);
  }

  public void zeroHeading() {
    Pose2d newPose = new Pose2d(getPose().getTranslation(), new Rotation2d());
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), newPose);
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
  // boolean doRejectUpdate = false;

  // swerveOdometry.update(getGyroYaw(), getModulePositions());

  // LimelightHelpers.SetRobotOrientation("limelight",
  // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
  // 0, 0);
  // LimelightHelpers.PoseEstimate mt2 =
  // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
  // if(Math.abs(pigeon.getRate()) > 360) // if our angular velocity is greater
  // than 360 degrees per second, ignore vision updates
  // {
  // doRejectUpdate = true;
  // }
  // if(mt2.tagCount == 0)
  // {
  // doRejectUpdate = true;
  // }
  // if(!doRejectUpdate)
  // {
  // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  // m_poseEstimator.addVisionMeasurement(
  // mt2.pose,
  // mt2.timestampSeconds);
  // }
  // }
  @Override
  public Command getCurrentCommand() {
    return currentCommand;
  }

  public Command asSubsystemCommand(Command command, String commandName) {
    command.setName(commandName);
    command.addRequirements(this);
    return command.beforeStarting(new InstantCommand(() -> currentCommand = command));
  }

  @Override
  public void subsystemPeriodic() {
    this.estimator.updateSwerve(getGyroYaw(), getModulePositions());
    var status = m_canBus.getStatus();
    SmartDashboard.putNumber("CANivore Load", status.BusUtilization * 100);

    swerveOdometry.update(getGyroYaw(), getModulePositions());
    field.setRobotPose(getPose());

    Logger.recordOutput("Estimator/states/Mystates", getModuleStates());
    Logger.recordOutput("Estimator/states/MyTargetStates", getTargetModuleStates());
    Logger.recordOutput("Estimator/Odometry", getOdometryPose());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Target Velocity",
          mod.getTargetState().speedMetersPerSecond);

      Logger.recordOutput(
          "Estimator/Mods/Mod " + mod.moduleNumber + " Cancoder", mod.getCANcoder().getDegrees());
      Logger.recordOutput(
          "Estimator/Mods/Mod " + mod.moduleNumber + " Integrated",
          mod.getPosition().angle.getDegrees());
      Logger.recordOutput(
          "Estimator/Mods/Mod " + mod.moduleNumber + " Velocity",
          mod.getState().speedMetersPerSecond);
      Logger.recordOutput(
          "Estimator/Mods/Mod " + mod.moduleNumber + " Target Velocity",
          mod.getTargetState().speedMetersPerSecond);
    }

    SmartDashboard.putNumber("Pigeon2 Yaw", pigeon.getYaw().getValueAsDouble());
  }
}
