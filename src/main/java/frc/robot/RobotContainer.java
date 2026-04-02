// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TargetConstants;
import frc.robot.subsystems.extendo.Extendo;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

        // Replace with CommandPS4Controller or CommandJoystick if needed
        final CommandXboxController driverXbox = new CommandXboxController(0);
        final CommandXboxController operatorXbox = new CommandXboxController(1);

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                        "swerve"));
        private final Shooter shooter = new Shooter();
        private final Extendo extendo = new Extendo();
        private final Intake intake = new Intake();
        private final Hopper hopper = new Hopper();

        // Establish a Sendable Chooser that will be able to be sent to the
        // SmartDashboard, allowing selection of desired auto
        private final SendableChooser<Command> autoChooser;

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> driverXbox.getLeftY() * -1,
                        () -> driverXbox.getLeftX() * -1)
                        .withControllerRotationAxis(() -> -driverXbox.getRightX())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        SwerveInputStream driveAim = driveAngularVelocity.copy()
                        .aimHeadingOffset(Rotation2d.k180deg)
                        .aimHeadingOffset(true)
                        .aimWhile(true);
        SwerveInputStream driveAimBlueHubInputs = driveAim.copy().aim(TargetConstants.blueHubPose);
        SwerveInputStream driveAimRedHubInputs = driveAim.copy().aim(TargetConstants.redHubPose);

        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(driverXbox::getRightX,
                                        driverXbox::getRightY)
                        .headingWhile(true);

        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                        2))
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8)
                        .allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        driverXbox.getRawAxis(
                                                        2) *
                                                        Math.PI)
                                        *
                                        (Math.PI *
                                                        2),
                                        () -> Math.cos(
                                                        driverXbox.getRawAxis(
                                                                        2) *
                                                                        Math.PI)
                                                        *
                                                        (Math.PI *
                                                                        2))
                        .headingWhile(true)
                        .translationHeadingOffset(true)
                        .translationHeadingOffset(Rotation2d.fromDegrees(
                                        0));

        Command fullShootBlindCommand() {
                return shooter.shootBlindCommand().until(shooter::isReadyToShoot)
                                .andThen((hopper.inCommand()));
        };

        Command fullShootHubCommand() {
                return shooter.shootHubCommand(drivebase::getDistanceToHub).until(shooter::isReadyToShoot)
                                .andThen((hopper.inCommand()));
        };

        Command agitateCommand() {
                return extendo.retractCommand().until(extendo::isAgitateRetracted)
                                .andThen(extendo.extendCommand().until(extendo::isAgitateExtend)).repeatedly();
        };

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                DriverStation.silenceJoystickConnectionWarning(true);

                // Create the NamedCommands that will be used in PathPlanner
                NamedCommands.registerCommand("Shoot",
                                fullShootBlindCommand().alongWith(agitateCommand()).withTimeout(10.0));

                // Have the autoChooser pull in all PathPlanner autos as options
                autoChooser = AutoBuilder.buildAutoChooser();

                // Set the default auto (do nothing)
                autoChooser.setDefaultOption("Do Nothing", Commands.none());

                // Put the autoChooser on the SmartDashboard
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                @SuppressWarnings("unused")
                Command driveAimHub = Commands.either(
                                drivebase.driveFieldOriented(driveAimRedHubInputs),
                                drivebase.driveFieldOriented(driveAimBlueHubInputs),
                                drivebase::isRedAlliance);

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                shooter.setDefaultCommand(shooter.stopCommand());
                extendo.setDefaultCommand(extendo.moveCommand(() -> operatorXbox.getLeftY()));
                intake.setDefaultCommand(intake.stopCommand());
                hopper.setDefaultCommand(hopper.stopCommand());

                driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                driverXbox.start().whileTrue(Commands.runOnce(drivebase::zeroGyro));
                driverXbox.leftTrigger().whileTrue(Commands.runEnd(
                                drivebase.setMaxSpeed(Constants.MAX_SPEED * 0.5),
                                drivebase::resetMaxSpeed));
                driverXbox.rightTrigger().whileTrue(driveAimHub);
                // TODO: configure drive to shoot pose commands
                // driverXbox.b().whileTrue(
                // drivebase.driveToPose(
                // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                // );

                operatorXbox.leftTrigger().whileTrue(shooter.shootBlindCommand());
                operatorXbox.leftBumper().whileTrue(agitateCommand());
                operatorXbox.rightTrigger()
                                .whileTrue(fullShootBlindCommand());
                operatorXbox.rightBumper()
                                .whileTrue(fullShootHubCommand());
                operatorXbox.a().whileTrue(intake.inCommand().alongWith(Commands.runEnd(
                                drivebase.setMaxSpeed(Constants.MAX_SPEED * 0.5),
                                drivebase::resetMaxSpeed)));
                operatorXbox.b().whileTrue(intake.outCommand());
                operatorXbox.x().whileTrue(hopper.inCommand());
                operatorXbox.y().whileTrue(hopper.outCommand());
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Pass in the selected auto from the SmartDashboard as our desired autnomous
                // commmand
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}
