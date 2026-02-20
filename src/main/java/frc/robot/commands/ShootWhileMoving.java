package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlignToPoseSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter.FlyWheel.FlyWheelSub;
import frc.robot.subsystems.Shooter.Hood.HoodSUB;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FlightTimeTable;
import frc.robot.util.ShotPrediction;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

public class ShootWhileMoving extends Command {
  private final Swerve swerve;
  private final PoseEstimator poseEstimator;
  private final HoodSUB hood;
  private final FlyWheelSub flywheel;
  private final Spindexer spindexer;
  private final AlignToPoseSubsystem alignSubsystem;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  // Constants for this command
  private static final double INDEX_SPEED = 0.35; // Matches Spin command usage

  public ShootWhileMoving(
      Swerve swerve,
      PoseEstimator poseEstimator,
      HoodSUB hood,
      FlyWheelSub flywheel,
      Spindexer spindexer,
      AlignToPoseSubsystem alignSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {

    this.swerve = swerve;
    this.poseEstimator = poseEstimator;
    this.hood = hood;
    this.flywheel = flywheel;
    this.spindexer = spindexer;
    this.alignSubsystem = alignSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    addRequirements(swerve, hood, flywheel, spindexer, alignSubsystem);
  }

  @Override
  public void execute() {
    // 1. Get Current Pose
    Pose2d currentPose = poseEstimator.getEstimatedPosition();

    // 2. Determine Hub Position based on Alliance
    Translation2d hubPosition = Constants.FieldConstants.HUB_CENTER_BLUE;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPosition = Constants.FieldConstants.HUB_CENTER_RED;
    }

    // 3. Calculate Distance to Hub (for Setpoints) & Set Subsystems
    double distanceToHub = currentPose.getTranslation().getDistance(hubPosition);

    flywheel.setTargetRPM(flywheel.getRPMFromDistance(distanceToHub));
    hood.setTargetArc(hood.getArcFromDistance(distanceToHub));

    // 4. Get Flight Time
    double flightTime = FlightTimeTable.get(distanceToHub);

    // 5. Get Field Relative Speeds
    // Swerve gives RobotRelative speeds. We need to convert to FieldRelative
    // manually or use helper.
    ChassisSpeeds robotRelativeSpeeds = swerve.getRobotRelativeSpeeds();

    // Convert RobotVelocity (vx, vy) to FieldVelocity by rotating by robot heading
    Translation2d robotLinearVelocity =
        new Translation2d(
            robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
    Translation2d fieldLinearVelocity = robotLinearVelocity.rotateBy(currentPose.getRotation());

    ChassisSpeeds fieldRelativeSpeeds =
        new ChassisSpeeds(
            fieldLinearVelocity.getX(),
            fieldLinearVelocity.getY(),
            robotRelativeSpeeds.omegaRadiansPerSecond);

    // Deadband speeds to reduce noise when standing still (Requested by user)
    if (Math.abs(fieldRelativeSpeeds.vxMetersPerSecond) < 0.1)
      fieldRelativeSpeeds.vxMetersPerSecond = 0;
    if (Math.abs(fieldRelativeSpeeds.vyMetersPerSecond) < 0.1)
      fieldRelativeSpeeds.vyMetersPerSecond = 0;

    // 6. Predict Future Position
    // futurePos = currentPos + (vx, vy) * flightTime
    Translation2d futurePos =
        ShotPrediction.predictFuturePosition(currentPose, fieldRelativeSpeeds, flightTime);

    // 7. Calculate Rotation Output using AlignToPoseSubsystem
    // We act "as if" we are at the future position but with current rotation,
    // asking the subsystem to face the hub from that position.
    Pose2d futurePoseForCalc = new Pose2d(futurePos, currentPose.getRotation());
    double rotationOutput = alignSubsystem.calculateRotationOutput(futurePoseForCalc, hubPosition);

    // Driver control for translation
    double xSpeed = translationXSupplier.getAsDouble();
    double ySpeed = translationYSupplier.getAsDouble();

    // Drive field relative
    swerve.drive(
        new Translation2d(xSpeed, ySpeed),
        rotationOutput,
        true, // fieldRelative
        false // isOpenLoop
        );

    // 8. Gating & Shooting
    boolean hoodReady = hood.isAtTarget();
    boolean flywheelReady = flywheel.isAtTarget();
    boolean aligned = alignSubsystem.atSetpoint();

    if (hoodReady && flywheelReady && aligned) {
      spindexer.SetMotor(INDEX_SPEED); // Feed
    } else {
      spindexer.SetMotor(0);
    }

  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true, false);
    spindexer.SetMotor(0);
    hood.stop();
    flywheel.stop();
  }
}
