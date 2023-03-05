// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import ca.warp7.frc2023.Constants.kControllers;
import ca.warp7.frc2023.commands.TeleopDriveCommand;
import ca.warp7.frc2023.commands.TeleopElevatorCommand;
import ca.warp7.frc2023.commands.TeleopFourbarCommand;
import ca.warp7.frc2023.commands.TeleopIntakeCommand;
import ca.warp7.frc2023.commands.SimpleConeAuto;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private final CommandXboxController primaryOperatorController =
            new CommandXboxController(kControllers.kDriverControllerID);
    private final CommandXboxController secondaryOperatorController =
            new CommandXboxController(kControllers.kOperatorControllerID);
    private final CommandXboxController technicianController =
            new CommandXboxController(kControllers.kTechnicianControllerID);

    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem = new SwerveDrivetrainSubsystem();
    private final FourbarSubsystem fourbarSubsystem = new FourbarSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        swerveDrivetrainSubsystem.setDefaultCommand(new TeleopDriveCommand(
                swerveDrivetrainSubsystem,
                () -> -primaryOperatorController.getLeftY(),
                () -> -primaryOperatorController.getLeftX(),
                () -> -primaryOperatorController.getRightX(),
                () -> primaryOperatorController.leftBumper().getAsBoolean()));

        intakeSubsystem.setDefaultCommand(new TeleopIntakeCommand(
                intakeSubsystem,
                () -> secondaryOperatorController.back().getAsBoolean(),
                () -> secondaryOperatorController.a().getAsBoolean(),
                () -> secondaryOperatorController.getRightTriggerAxis(),
                () -> secondaryOperatorController.getLeftTriggerAxis(),
                () -> secondaryOperatorController.getRightTriggerAxis(),
                () -> secondaryOperatorController.getRightY()));

        fourbarSubsystem.setDefaultCommand(new TeleopFourbarCommand(fourbarSubsystem, () -> secondaryOperatorController.getLeftY()));

        elevatorSubsystem.setDefaultCommand(new TeleopElevatorCommand(elevatorSubsystem, () -> secondaryOperatorController.getRightY()));

        configureBindings();

        autoChooser.setDefaultOption("Default Auto", Commands.print("Default Auto"));
        autoChooser.addOption("Simple cone auto", new SimpleConeAuto());
    }

    private void configureBindings() {
        primaryOperatorController.back().onTrue(new InstantCommand(swerveDrivetrainSubsystem::zeroGyro));
        primaryOperatorController
                .b()
                .onTrue(new InstantCommand(swerveDrivetrainSubsystem::resetSwerveModulesToAbsolute));

        secondaryOperatorController.x().onTrue(fourbarSubsystem.setPosition(0));
        secondaryOperatorController.y().onTrue(fourbarSubsystem.setPosition(25));
        secondaryOperatorController.b().onTrue(fourbarSubsystem.setPosition(57.5));

        secondaryOperatorController.povUp().onTrue(intakeSubsystem.setTalonPivotSetPoint(-15));

        

        // secondaryOperatorController.povUp().onTrue(elevatorSubsystem.highGoal());
        secondaryOperatorController.povLeft().onTrue(elevatorSubsystem.startPosition());
        secondaryOperatorController.povDown().onTrue(elevatorSubsystem.mediumGoal());

        secondaryOperatorController.leftBumper().onTrue(intakeSubsystem.setTalonPivotSetPoint(-5));
        secondaryOperatorController.rightBumper().onTrue(intakeSubsystem.setTalonPivotSetPoint(-25));
    }

    public Command getAutonomousCommand() {
        // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));
        // return Commands.print("No auton");
        return autoChooser.getSelected();
    }
}
