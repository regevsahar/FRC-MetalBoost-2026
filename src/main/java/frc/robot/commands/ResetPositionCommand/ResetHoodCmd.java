package frc.robot.commands.ResetPositionCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;

public class ResetHoodCmd extends Command {
    private final HoodSUB hood;

    public ResetHoodCmd(HoodSUB hood) {
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
