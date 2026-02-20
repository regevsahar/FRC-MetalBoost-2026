package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.DriveToPoseUtil;
import frc.lib.util.HeightSpeedReduction;
import frc.lib.util.MapFiltering.FieldGridLoader;
import frc.lib.util.MapFiltering.GridMap;
import frc.robot.autos.AutoChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
        /* Controllers */
        private final Joystick driver = new Joystick(0);

        /* Drive Controls */
        private final int translationAxis = XboxController.Axis.kLeftY.value;
        private final int strafeAxis = XboxController.Axis.kLeftX.value;
        private final int rotationAxis = XboxController.Axis.kRightX.value;

        /* Driver Buttons */
        private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
        private final JoystickButton lowerSwerveSpeed = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);
        private final JoystickButton higherSwerveSpeed = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        private final JoystickButton resetPoseEstimator = new JoystickButton(driver, XboxController.Button.kA.value);
        private final JoystickButton AlignToPose = new JoystickButton(driver, XboxController.Button.kX.value);
        private final JoystickButton followPath = new JoystickButton(driver, XboxController.Button.kB.value);


        AutoChooser autoChooser;

        /* Subsystems */

        private HeightSpeedReduction heightSpeedReduction = HeightSpeedReduction.getInstance();

        public final PoseEstimator poseEstimator = new PoseEstimator();
        public final LimelightSubsystem limelight = new LimelightSubsystem(Constants.VisionConstants.limelight3name);
        public final LimelightSubsystem limelight2 = new LimelightSubsystem(Constants.VisionConstants.limelight4name);
        public final Swerve s_Swerve = new Swerve(poseEstimator);
        public final AlignToPoseSubsystem AlignToPoseSub = new AlignToPoseSubsystem();

        public final GridMap fieldGrid;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                fieldGrid = FieldGridLoader.load("FieldGrid.json");

                s_Swerve.setDefaultCommand(
                                new TeleopSwerve(
                                                s_Swerve,
                                                () -> -driver.getRawAxis(translationAxis),
                                                () -> -driver.getRawAxis(strafeAxis),
                                                () -> -driver.getRawAxis(rotationAxis),
                                                () -> true,
                                                heightSpeedReduction.getSpeedSupplier()));

                // elevatorSUB.setDefaultCommand(new ManualElevatorControll(elevatorSUB, () ->
                // operator.getRawAxis(elevatorAxis)));

                // Configure the button bindings
                configureButtonBindings();
                registerPathPlannerCommands();


        }

        private void configureButtonBindings() {
                /* Driver Buttons */
                zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
                lowerSwerveSpeed.whileTrue(new TeleopSwerve(
                                s_Swerve,
                                () -> -driver.getRawAxis(translationAxis),
                                () -> -driver.getRawAxis(strafeAxis),
                                () -> -driver.getRawAxis(rotationAxis),
                                () -> true,
                                () -> 0.4));

                higherSwerveSpeed.whileTrue(new TeleopSwerve(
                                s_Swerve,
                                () -> -driver.getRawAxis(translationAxis),
                                () -> -driver.getRawAxis(strafeAxis),
                                () -> -driver.getRawAxis(rotationAxis),
                                () -> true,
                                () -> 0.85));

                resetPoseEstimator.onTrue(new InstantCommand(() -> poseEstimator.sEstimator.resetPosition(
                                s_Swerve.getGyroYaw(),
                                s_Swerve.getModulePositions(),
                                new Pose2d(0, 0, new Rotation2d()))));


                AlignToPose.toggleOnTrue(new AlignToPoseCommand(s_Swerve, AlignToPoseSub,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis)));
                               
                followPath.whileTrue(new DeferredCommand(
                        () -> new DriveToPoseUtil().driveToPoseHolonomic(
                                poseEstimator.getEstimatedPosition(),
                                new Pose2d(8.5, 3.5, Rotation2d.fromDegrees(180)), // 8, 4.5, 0 ?????????????
                                .5,
                                .5
                                ),
                                Set.of(s_Swerve)
                        ));

        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelectedAuto();
        }

        void registerPathPlannerCommands() {
        }

}