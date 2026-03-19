package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.IntakeCommands.OpenIntakeCmd;
import frc.robot.commands.IntakeCommands.RollToEjectBallsCmd;
import frc.robot.commands.LedCommand;
import frc.robot.commands.LedCommand.Mode;
import frc.robot.subsystems.Conveyance.ConveyanceRollerSub;
import frc.robot.subsystems.Intake.IntakeRollersSub;
import frc.robot.subsystems.Intake.IntakeSub;

public class EjectBallsAutomationCmd extends ParallelCommandGroup {
  public EjectBallsAutomationCmd(
      IntakeRollersSub intakeRollers, ConveyanceRollerSub rollers, IntakeSub intake) {

    addCommands(
        new RollToEjectBallsCmd(intakeRollers),
        new RollersBackwardsCmd(rollers),
        new OpenIntakeCmd(intake),
        new LedCommand(Mode.GRADIENT, Color.kRed, Color.kYellow));
  }
}
