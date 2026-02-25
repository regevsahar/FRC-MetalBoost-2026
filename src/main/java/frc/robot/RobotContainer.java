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
import frc.lib.util.PathPlannerUtil;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.FlywheelHoodIntegrationCommand;
import frc.robot.commands.Shooter.HoodCommand;
import frc.robot.commands.Shooter.ShootCommand;
import frc.robot.commands.Shooter.SpinDexser.SpinCommand;
import frc.robot.commands.Swerve.TeleopSwerveCommand;
import frc.robot.commands.Vision.ShootWhileMovingCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSimulation;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOTalonFX;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.SpinDexser.SpindexerSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.AlignToPoseSubsystem;
import frc.robot.subsystems.Vision.LimelightSubsystem;
import frc.robot.subsystems.Vision.PoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;

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
//   private final JoystickButton followPath = new JoystickButton(driver, XboxController.Button.kB.value);

  private final JoystickButton GoToNearestBranch = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton resetPoseEstimator =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton shootWhileMoving =
      new JoystickButton(driver, XboxController.Button.kX.value);

  AutoChooser autoChooser;

  /* Subsystems */

  private HeightSpeedReduction heightSpeedReduction = HeightSpeedReduction.getInstance();
  private final FlyWheelSub shooter;
  private final HoodSUB hood;
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final IntakeSubsystem s_intake = new IntakeSubsystem();
  public final PoseEstimator poseEstimator = new PoseEstimator();
  public final LimelightSubsystem limelight =
      new LimelightSubsystem(CameraConstants.limelight3name);
  public final LimelightSubsystem limelight2 =
      new LimelightSubsystem(CameraConstants.limelight4name);
  public final SwerveSubsystem s_Swerve = new SwerveSubsystem(poseEstimator);
  public final AlignToPoseSubsystem AlignToPoseSub = new AlignToPoseSubsystem();

  /// * operation Buttons */
  private final Trigger shoot = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton spin =
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton spinAnotherSide =
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton flywheelHoodAutoCommand =
      new JoystickButton(operator, XboxController.Button.kStart.value);
  private final Trigger intake = new JoystickButton(operator, XboxController.Button.kB.value);
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
        new TeleopSwerveCommand(
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

    shoot.whileTrue(new ShootCommand(shooter));
    spin.whileTrue(new SpinCommand(spindexer, -0.35));
    spinAnotherSide.whileTrue(new SpinCommand(spindexer, 0.35));
    intake.whileTrue(new IntakeCommand(s_intake, 0.45));
    hoodCommand.whileTrue(new HoodCommand(hood));
    flywheelHoodAutoCommand.whileTrue(new FlywheelHoodIntegrationCommand(shooter, hood));

    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    lowerSwerveSpeed.whileTrue(
        new TeleopSwerveCommand(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> true,
            () -> 0.4));

    higherSwerveSpeed.whileTrue(
        new TeleopSwerveCommand(
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

    // Shoot While Moving
    shootWhileMoving.whileTrue(
        new ShootWhileMovingCommand(
            s_Swerve,
            poseEstimator,
            AlignToPoseSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)));

    GoToNearestBranch.onTrue(
        PathPlannerUtil.GoToNearesPosition(
            poseEstimator.getEstimatedPosition(),
            Constants.SwerveConstants.constraints)
        );

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedAuto();
  }

  void registerPathPlannerCommands() {}
}
