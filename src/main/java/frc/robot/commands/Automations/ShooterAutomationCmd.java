package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.commands.ConveyanceCommands.ConveyanceWheelsCmd;
import frc.robot.commands.ConveyanceCommands.RollersBackwardsCmd;
import frc.robot.commands.ShooterCommands.HoodCmd;
import frc.robot.commands.ShooterCommands.ShooterCmd;

import frc.robot.subsystems.Conveyance.ConveyanceSub;
import frc.robot.subsystems.Conveyance.RollersSub;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class ShooterAutomationCmd extends SequentialCommandGroup {

  public ShooterAutomationCmd(
      FlyWheelSub shooter,
      HoodSUB hood,
      ConveyanceSub conveyanceWheels,
      RollersSub rollers) {

    addCommands(

        new ParallelCommandGroup(
            new ShooterCmd(shooter),
            new HoodCmd(hood),
            new ConveyanceWheelsCmd(conveyanceWheels),
            new RollersBackwardsCmd(rollers)),

        new WaitUntilCommand(() -> !conveyanceWheels.hasBall()),

        new WaitCommand(2)
    );
  }
}