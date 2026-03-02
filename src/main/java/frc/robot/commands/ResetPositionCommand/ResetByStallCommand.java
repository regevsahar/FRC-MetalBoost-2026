package frc.robot.commands.ResetPositionCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ResetByStallCommand extends Command {

    private final DoubleConsumer setSpeed;
    private final Runnable stop;
    private final Runnable resetPosition;
    private final BooleanSupplier isStalling;
    private final double speed;

    public ResetByStallCommand(
            Subsystem subsystem,
            DoubleConsumer setSpeed,
            Runnable stop,
            Runnable resetPosition,
            BooleanSupplier isStalling,
            double speed) 
    {

        this.setSpeed = setSpeed;
        this.stop = stop;
        this.resetPosition = resetPosition;
        this.isStalling = isStalling;
        this.speed = speed;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        setSpeed.accept(speed);
    }

    @Override
    public void end(boolean interrupted) {
        stop.run();
        resetPosition.run();
    }

    @Override
    public boolean isFinished() {
        return isStalling.getAsBoolean();
    }
}
