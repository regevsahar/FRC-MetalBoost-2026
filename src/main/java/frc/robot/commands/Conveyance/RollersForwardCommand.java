package frc.robot.commands.Conveyance;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyance.RollersSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class RollersForwardCommand extends Command {

    private final RollersSubsystem rollers;

    public RollersForwardCommand(RollersSubsystem rollers) {
        this.rollers = rollers;
        addRequirements(rollers);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        rollers.setSpeed(ShooterConstants.kRollersForwardSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        rollers.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
