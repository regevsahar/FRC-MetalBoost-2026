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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.HeightSpeedReduction;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
  // private final JoystickButton GoToNearestBranch = new JoystickButton(driver,
  // XboxController.Button.kB.value);
  private final JoystickButton resetPoseEstimator =
      new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton shootWhileMoving =
      new JoystickButton(driver, XboxController.Button.kX.value);

  AutoChooser autoChooser;

  /* Subsystems */

  private HeightSpeedReduction heightSpeedReduction = HeightSpeedReduction.getInstance();
  private final FlyWheelSub shooter;
  private final HoodSUB hood;
  public final PoseEstimator poseEstimator = new PoseEstimator();
  public final LimelightSubsystem limelight = new LimelightSubsystem("limelight-three");
  public final LimelightSubsystem limelight2 = new LimelightSubsystem("limelight-four");
  public final Swerve s_Swerve = new Swerve(poseEstimator);
  public final AlignToPoseSubsystem AlignToPoseSub = new AlignToPoseSubsystem();

  // ── SysId Buttons (Operator Controller) ─────────────────────────────────
  // Hood │ D-Pad Up → quasistatic forward
  // │ D-Pad Down → quasistatic reverse
  // │ D-Pad Left → dynamic forward
  // │ D-Pad Right → dynamic reverse

  //private final Trigger hoodSysIdQuasiF = new Trigger(() -> operator.getPOV() == 0);
  //private final Trigger hoodSysIdQuasiR = new Trigger(() -> operator.getPOV() == 180);
  //private final Trigger hoodSysIdDynF = new Trigger(() -> operator.getPOV() == 270);
  //private final Trigger hoodSysIdDynR = new Trigger(() -> operator.getPOV() == 90);

  // Flywheel │ Y → quasistatic forward
  // │ A → quasistatic reverse
  // │ X → dynamic forward
  // │ B → dynamic reverse
  private final JoystickButton hoodCommand =
      new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton hoodSysIdQuasiR =
      new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton hoodSysIdDynF =
      new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton hoodSysIdDynR =
      new JoystickButton(operator, XboxController.Button.kB.value);

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

    // ── Hood SysId ──────────────────────────────────────────────────────────
    hoodCommand.whileTrue(new HoodCommand(hood));
    hoodSysIdQuasiR.whileTrue(hood.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    hoodSysIdDynF.whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kForward));
    hoodSysIdDynR.whileTrue(hood.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // ── Flywheel SysId ──────────────────────────────────────────────────────
    //flywheelSysIdQuasiF.whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    //flywheelSysIdQuasiR.whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //flywheelSysIdDynF.whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //flywheelSysIdDynR.whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

    // Shoot While Moving
    shootWhileMoving.whileTrue(
        new ShootWhileMoving(
            s_Swerve,
            poseEstimator,
            AlignToPoseSub,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedAuto();
  }

  void registerPathPlannerCommands() {}
}
