package frc.robot.commands.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.SwerveSub;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerveCmd extends Command {
  private SwerveSub s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private DoubleSupplier speedReductionSup;

  public TeleopSwerveCmd(
      SwerveSub s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.speedReductionSup = () -> 1.0;
  }

  public TeleopSwerveCmd(
      SwerveSub s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      DoubleSupplier speedReductionSup) {
    this.s_Swerve = s_Swerve;

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.speedReductionSup = speedReductionSup;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal =
        MathUtil.applyDeadband(
            translationSup.getAsDouble() * speedReductionSup.getAsDouble(),
            Constants.stickDeadband);
    double strafeVal =
        MathUtil.applyDeadband(
            strafeSup.getAsDouble() * speedReductionSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal =
        MathUtil.applyDeadband(
            rotationSup.getAsDouble() * speedReductionSup.getAsDouble(), Constants.stickDeadband);

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        true,
        true);
  }
}
