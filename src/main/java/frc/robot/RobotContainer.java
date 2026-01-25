package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton lowerSwerveSpeed = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton higherSwerveSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* operation Buttons */
  private final Trigger shoot = new Trigger(() -> operator.getRawAxis(XboxController.Axis.kRightY.value) > 0.1);
  private final JoystickButton spin = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton intake = new JoystickButton(operator, XboxController.Button.kB.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Spindexer spindexer = new Spindexer();
  private final Intake s_intake = new Intake();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

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

  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
    lowerSwerveSpeed.whileTrue(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> true,
        () -> 0.4));

    higherSwerveSpeed.whileTrue(new TeleopSwerve(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis),
        () -> true,
        () -> 0.85));

    shoot.whileTrue(new Shoot(shooter, () -> operator.getRawAxis(XboxController.Axis.kRightTrigger.value)));
    spin.whileTrue(new Spin(spindexer, -0.8));
    intake.whileTrue(new IntakeCommand(s_intake, 0.95));

  }

  void registerPathPlannerCommands() {
  }

}