package frc.robot.commands.ResetPositionCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood.HoodSub;

public class ResetHoodCommand extends Command {
    private final HoodSub hood;

    public ResetHoodCommand(HoodSub hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hood.setSpeed(-0.1); // :TODO: "tune" this
    }

    @Override
    public void end(boolean interrupted) {
        hood.stop();
        hood.resetPosition();
    }

    @Override
    public boolean isFinished() {
        return hood.isStalling();
    }
}
