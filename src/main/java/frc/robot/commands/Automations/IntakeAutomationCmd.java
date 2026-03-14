package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCommands.OpenIntakeCmd;
import frc.robot.commands.IntakeCommands.RollToInsertBallsCmd;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutomationCmd extends ParallelCommandGroup {
  public IntakeAutomationCmd(
      IntakeSub intakeMovement, IntakeRollersSub rollers) { // ,RollersSub conveyanceRollers) {

    addCommands(new OpenIntakeCmd(intakeMovement), new RollToInsertBallsCmd(rollers));
    // RollersBackwardsCmd(conveyanceRollers),
  }
}
