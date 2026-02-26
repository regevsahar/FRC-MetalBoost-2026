package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.HeightSpeedReduction;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.lib.util.PathPlannerUtil;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.Swerve.TeleopSwerveCommand;
import frc.robot.commands.Vision.ShootWhileMovingCommand;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSimulation;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOTalonFX;
import frc.robot.subsystems.Shooter.Hood.HoodSub;
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

  AutoChooser autoChooser;

  /* Subsystems */

  private HeightSpeedReduction heightSpeedReduction = HeightSpeedReduction.getInstance();
  public final PoseEstimator poseEstimator = new PoseEstimator();
  public final LimelightSubsystem limelight =
      new LimelightSubsystem(CameraConstants.limelight3name);
  public final LimelightSubsystem limelight2 =
      new LimelightSubsystem(CameraConstants.limelight4name);
  public final AlignToPoseSubsystem AlignToPoseSub = new AlignToPoseSubsystem();
  private final FlyWheelSub shooter;
  private final HoodSub hood;
  private final IntakeSub s_intake = new IntakeSub();
  private final IntakeRollersSub s_intakeRollers = new IntakeRollersSub();
  public final SwerveSubsystem s_Swerve = new SwerveSubsystem(poseEstimator);

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton lowerSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton higherSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  // private final JoystickButton followPath = new JoystickButton(driver,
  // XboxController.Button.kB.value);

  private final JoystickButton GoToNearestBranch =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton resetPoseEstimator =
      new JoystickButton(driver, XboxController.Button.kA.value);

  /// * operation Buttons */
  private final JoystickButton shootWhileMoving =
      new JoystickButton(operator, XboxController.Button.kX.value);

  public final GridMap fieldGrid;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FlyWheelIO shooterIO =
        RobotBase.isSimulation() ? new FlyWheelSimulation() : new FlyWheelIOTalonFX();

    shooter = new FlyWheelSub(shooterIO);

    HoodIO hoodIO = RobotBase.isSimulation() ? new HoodIOSim() : new HoodIOTalonFX();

    hood = new HoodSub(hoodIO);
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
            poseEstimator.getEstimatedPosition(), Constants.SwerveConstants.constraints));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedAuto();
  }

  void registerPathPlannerCommands() {}
}
