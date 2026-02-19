package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.HeightSpeedReduction;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.lib.util.PathPlannerUtil;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.commands.IntakeCommand.CloseIntake;
import frc.robot.commands.IntakeCommand.OpenIntake;
import frc.robot.commands.Shooter.FlywheelHoodIntegrationCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSimulation;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOTalonFX;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton lowerSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton higherSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton GoToNearestBranch =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton resetPoseEstimator =
      new JoystickButton(driver, XboxController.Button.kA.value);

  AutoChooser autoChooser;

  /* Subsystems */

  private HeightSpeedReduction heightSpeedReduction = HeightSpeedReduction.getInstance();
  private final FlyWheelSub shooter;
  private final HoodSUB hood;
  private final Spindexer spindexer = new Spindexer();
  private final Intake s_intake = new Intake();
  public final PoseEstimator poseEstimator = new PoseEstimator();
  public final LimelightSubsystem limelight = new LimelightSubsystem("limelight");
  public final LimelightSubsystem limelight2 = new LimelightSubsystem("limelight2");
  public final Swerve s_Swerve = new Swerve(poseEstimator);

  /// * operation Buttons */
  private final Trigger shoot = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton spin =
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton spinAnotherSide =
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton flywheelHoodAutoCommand =
      new JoystickButton(operator, XboxController.Button.kStart.value);
  private final Trigger OpenIntake = new JoystickButton(operator, XboxController.Button.kB.value);
  private final Trigger CloseIntake = new JoystickButton(operator, XboxController.Button.kA.value);
  private final Trigger hoodCommand = new JoystickButton(operator, XboxController.Button.kY.value);

  public final GridMap fieldGrid;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FlyWheelIO shooterIO =
        RobotBase.isSimulation() ? new FlyWheelSimulation() : new FlyWheelIOTalonFX();

    shooter = new FlyWheelSub(shooterIO);

    HoodIO hoodIO = RobotBase.isSimulation() ? new HoodIOSim() : new HoodIOTalonFX();

    hood = new HoodSUB(hoodIO);
    fieldGrid = FieldGridLoader.load("FieldGrid.json");

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> true,
            heightSpeedReduction.getSpeedSupplier()));

    // Configure the button bindings
    configureButtonBindings();
    registerPathPlannerCommands();
  }

  private void configureButtonBindings() {

    shoot.whileTrue(new Shoot(shooter));
    spin.whileTrue(new Spin(spindexer, -0.35));
    spinAnotherSide.whileTrue(new Spin(spindexer, 0.35));
    OpenIntake.whileTrue(new OpenIntake(s_intake));
    CloseIntake.whileTrue(new CloseIntake(s_intake));
    hoodCommand.whileTrue(new HoodCommand(hood));
    flywheelHoodAutoCommand.whileTrue(new FlywheelHoodIntegrationCommand(shooter, hood));

    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    lowerSwerveSpeed.whileTrue(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> true,
            () -> 0.4));

    higherSwerveSpeed.whileTrue(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> true,
            () -> 0.85));

    resetPoseEstimator.onTrue(
        new InstantCommand(
            () ->
                poseEstimator.sEstimator.resetPosition(
                    s_Swerve.getGyroYaw(),
                    s_Swerve.getModulePositions(),
                    new Pose2d(0, 0, new Rotation2d()))));

    GoToNearestBranch.onTrue(
        PathPlannerUtil.GoToNearestBranch(
            poseEstimator.getEstimatedPosition(), Constants.SwerveConstants.constraints));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedAuto();
  }

  void registerPathPlannerCommands() {}
}
