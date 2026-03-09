package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.util.LedController;
import frc.robot.commands.IntakeCommands.OpenIntakeCmd;
import frc.robot.commands.IntakeCommands.RollToInsertBallsCmd;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;

public class InsetBallsAutomationCmd extends ParallelCommandGroup {
  public InsetBallsAutomationCmd(IntakeSub intakeMovement, IntakeRollersSub rollers) {

    addCommands(
        new OpenIntakeCmd(intakeMovement),
        new RollToInsertBallsCmd(rollers),
        new FunctionalCommand(
            () -> LedController.getInstance().startGradient(Color.kYellow, Color.kLightYellow),
            () -> {},
            (interrupted) -> LedController.getInstance().stopGradient(),
            () -> false));
  }
}
