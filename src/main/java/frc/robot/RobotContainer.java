package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIO;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelIOTalonFX;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSimulation;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;

public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // private final int elevatorAxis = PS4Controller.Axis.kRightY.value;
  // private final int shooterAxis = PS4Controller.Axis.kLeftY.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton lowerSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton higherSwerveSpeed =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* operation Buttons */
  private final Trigger shoot = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton spin =
      new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton spinAnotherSide =
      new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton intake =
      new JoystickButton(operator, XboxController.Button.kB.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final FlyWheelSub shooter;
  private final Spindexer spindexer = new Spindexer();
  private final Intake s_intake = new Intake();

  // auto chooser
  AutoChooser autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    FlyWheelIO shooterIO =
        RobotBase.isSimulation() ? new FlyWheelSimulation() : new FlyWheelIOTalonFX();

    shooter = new FlyWheelSub(shooterIO);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> true,
            () -> 0.6));

    configureButtonBindings();
    registerPathPlannerCommands();

    autoChooser =
        new AutoChooser(
            new PathPlannerAuto("Line Auto"),
            new PathPlannerAuto("Auto To Center"),
            new PathPlannerAuto("line"),
            // new PathPlannerAuto("New New Auto"),
            new PathPlannerAuto("Auto To Left"),
            new PathPlannerAuto("Auto To Right"),
            // new PathPlannerAuto("3 coral from right 1.0"),
            // new PathPlannerAuto("3 coral from right 2.0"),
            // new PathPlannerAuto("Dis3Al"),
            new PathPlannerAuto("3CoralLeft"),
            new PathPlannerAuto("3CoralRight"),
            new PathPlannerAuto("1CoralCenter"));
  }

  private void configureButtonBindings() {
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

    shoot.whileTrue(new Shoot(shooter));
    spin.whileTrue(new Spin(spindexer, -0.35));
    spinAnotherSide.whileTrue(new Spin(spindexer, 0.35));
    intake.whileTrue(new IntakeCommand(s_intake, 0.45));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelectedAuto();
  }

  void registerPathPlannerCommands() {}
}
