// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.LimelightHelpers;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    FollowPathCommand.warmupCommand().schedule();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    Rotation2d currentGyro = m_robotContainer.s_Swerve.getGyroYaw();
    double distanceFromTag = LimelightHelpers.getTargetPose_CameraSpace("limelight-three")[0];
    double distanceFromTag2 = LimelightHelpers.getTargetPose_CameraSpace("limelight-four")[0];

    Pose2d robotPose = m_robotContainer.poseEstimator.getEstimatedPosition();
    boolean isInRedZone = m_robotContainer.fieldGrid.onForbiddenArea(robotPose);
    Logger.recordOutput("Grid/IsRedZone", isInRedZone);

    var index = m_robotContainer.fieldGrid.poseToGrid(robotPose);
    Logger.recordOutput("Grid/Row", index.row());
    Logger.recordOutput("Grid/Col", index.col());

    m_robotContainer.poseEstimator.updateSwerve(
        currentGyro,
        m_robotContainer.s_Swerve.getModulePositions());

    m_robotContainer.limelight.SetHeading(m_robotContainer.poseEstimator.getCorrectedHeading(currentGyro));

    Optional<LimelightHelpers.PoseEstimate> llestimateMT2 = m_robotContainer.limelight.getMegaTag2Pose();
    Optional<LimelightHelpers.PoseEstimate> llestimateMT1 = m_robotContainer.limelight.getMegaTag1Pose();

    Optional<Pose2d> llPoseMT2 = Optional.empty();
    Optional<Pose2d> llPoseMT1 = Optional.empty();

    Optional<LimelightHelpers.PoseEstimate> ll2estimateMT2 = m_robotContainer.limelight2.getMegaTag2Pose();
    Optional<LimelightHelpers.PoseEstimate> ll2estimateMT1 = m_robotContainer.limelight2.getMegaTag1Pose();

    Optional<Pose2d> ll2PoseMT2 = Optional.empty();
    Optional<Pose2d> ll2PoseMT1 = Optional.empty();

    Optional<Rotation2d> gyroYawAtTimeStamp = Optional.empty();

    if (llestimateMT2.isPresent() && llestimateMT1.isPresent()) {
      llPoseMT2 = Optional.of(llestimateMT2.get().pose);
      llPoseMT1 = Optional.of(llestimateMT1.get().pose);
      gyroYawAtTimeStamp = m_robotContainer.poseEstimator.getGyroYawAtTimeStamp(llestimateMT2.get().timestampSeconds);
    }

    if (ll2estimateMT2.isPresent() && ll2estimateMT1.isPresent()) {
      ll2PoseMT2 = Optional.of(ll2estimateMT2.get().pose);
      ll2PoseMT1 = Optional.of(ll2estimateMT1.get().pose);
      gyroYawAtTimeStamp = m_robotContainer.poseEstimator.getGyroYawAtTimeStamp(ll2estimateMT2.get().timestampSeconds);
    }

    if (m_robotContainer.limelight.hasTarget()) {
      if (llPoseMT2.isPresent() && gyroYawAtTimeStamp.isPresent()) {
        m_robotContainer.poseEstimator.updateHeadingOffset(
            gyroYawAtTimeStamp.get(),
            llPoseMT1.get().getRotation());

        m_robotContainer.poseEstimator.updateVision(
            llestimateMT2.get(),
            gyroYawAtTimeStamp.get(),
            distanceFromTag

        );
      }
    }

    if (m_robotContainer.limelight2.hasTarget()) {
      if (ll2PoseMT2.isPresent() && gyroYawAtTimeStamp.isPresent()) {
        m_robotContainer.poseEstimator.updateHeadingOffset(
            gyroYawAtTimeStamp.get(),
            ll2PoseMT1.get().getRotation());

        m_robotContainer.poseEstimator.updateVision(
            ll2estimateMT2.get(),
            gyroYawAtTimeStamp.get(),
            distanceFromTag2

        );
      }
    }

    if (isInRedZone)
      Constants.PoseEstimator.OdometryFactor = 2;
    else
      Constants.PoseEstimator.OdometryFactor = 1;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
