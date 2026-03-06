package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ResetPositionCommand.ResetHoodCmd;
import frc.robot.commands.ResetPositionCommand.ResetIntakeCmd;
import frc.robot.subsystems.Intake.IntakeSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetSubsystemsAutomationCmd extends ParallelCommandGroup {
  public ResetSubsystemsAutomationCmd(HoodSUB hood, IntakeSub intake) {

    addCommands(new ResetHoodCmd(hood), new ResetIntakeCmd(intake));
  }
}
