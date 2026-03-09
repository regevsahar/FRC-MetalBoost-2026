package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.IntakeCommands.RollToEjectBallsCmd;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Intake.IntakeRollersSub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EjectBallAutomation extends ParallelCommandGroup {
  public EjectBallAutomation(IntakeRollersSub intakeRollers, ConveyanceRollerSub rollers) {

    addCommands(new RollToEjectBallsCmd(intakeRollers), new RollersBackwardsCmd(rollers));
  }
}
