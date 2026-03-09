package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.LedCommand;
import frc.robot.commands.ResetPositionCommand.ResetHoodCmd;
import frc.robot.commands.ResetPositionCommand.ResetIntakeCmd;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.commands.LedCommand.Mode;

public class ResetSubsystemsAutomationCmd extends ParallelCommandGroup {
  public ResetSubsystemsAutomationCmd(HoodSUB hood, IntakeSub intake) {

    addCommands(
        new ResetHoodCmd(hood),
        new ResetIntakeCmd(intake),
        new LedCommand(Mode.FLASH, new Color(255, 100, 100)));
        /* 
        new FunctionalCommand(
            () -> LedController.getInstance().startFlashing(new Color(255, 100, 100)),
            () -> {},
            (interrupted) -> LedController.getInstance().stopFlashing(),
            () -> false));
        */
    // addCommands(
    // new ResetByStallCmd(hood,hood::setSpeed,hood::stop, hood::resetPosition,hood::isStalling,
    // -0.2),
    // new ResetByStallCmd(intake,intake::SetMotor,intake::stop,
    // intake::resetPosition,intake::isStalling, -3),
    // new FunctionalCommand(
    //     () -> LedController.getInstance().startFlashing(new Color(255, 100, 100)),
    //     () -> {},
    //     (interrupted) -> LedController.getInstance().stopFlashing(),
    //     () -> false));
  }
}
