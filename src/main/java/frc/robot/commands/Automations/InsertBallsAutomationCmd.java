package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.LedCommand;
import frc.robot.commands.IntakeCommands.OpenIntakeCmd;
import frc.robot.commands.IntakeCommands.RollToInsertBallsCmd;
import frc.robot.commands.LedCommand.Mode;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;

public class InsertBallsAutomationCmd extends ParallelCommandGroup {
  public InsertBallsAutomationCmd(IntakeSub intakeMovement, IntakeRollersSub rollers) {

    addCommands(
        new OpenIntakeCmd(intakeMovement),
        new RollToInsertBallsCmd(rollers),
        new LedCommand(Mode.GRADIENT, Color.kYellow, Color.kLightYellow));
  }
}
