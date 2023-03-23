// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import ca.warp7.frc2023.Constants.kControllers;
import ca.warp7.frc2023.commands.BalanceCommand;
import ca.warp7.frc2023.commands.SetPointCommands;
import ca.warp7.frc2023.commands.TeleopDriveCommand;
import ca.warp7.frc2023.commands.TeleopElevatorCommand;
import ca.warp7.frc2023.commands.TeleopFourbarCommand;
import ca.warp7.frc2023.commands.TeleopIntakeCommand;
import ca.warp7.frc2023.commands.auton.MobilityCone;
import ca.warp7.frc2023.commands.auton.SimpleConeAuto;
import ca.warp7.frc2023.commands.auton.TestAuto;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
                () -> primaryOperatorController.leftBumper().getAsBoolean(),
                () -> primaryOperatorController.rightBumper().getAsBoolean()));

        intakeSubsystem.setDefaultCommand(new TeleopIntakeCommand(
                intakeSubsystem,
                () -> secondaryOperatorController.getRightTriggerAxis(),
                () -> secondaryOperatorController.getLeftTriggerAxis(),
                () -> secondaryOperatorController
                        .leftBumper()
                        .or(secondaryOperatorController.rightBumper())
                        .getAsBoolean()));

        fourbarSubsystem.setDefaultCommand(
                new TeleopFourbarCommand(fourbarSubsystem, () -> secondaryOperatorController.getLeftY()));

        elevatorSubsystem.setDefaultCommand(
                new TeleopElevatorCommand(elevatorSubsystem, () -> -secondaryOperatorController.getRightY()));

        configureBindings();

        configureAuto();
    }

    private void configureAuto() {
        autoChooser.setDefaultOption("NO AUTO!", Commands.print("No auto selected"));
        autoChooser.addOption("Simple cone auto", new SimpleConeAuto(intakeSubsystem));
        autoChooser.addOption("Cone and mobility", new MobilityCone(intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption("Daniel's Test", new TestAuto(swerveDrivetrainSubsystem));
        SmartDashboard.putData("autoChooser", autoChooser);
    }

    private void configureBindings() {
        primaryOperatorController.back().onTrue(new InstantCommand(swerveDrivetrainSubsystem::zeroGyro));
        primaryOperatorController
                .leftStick()
                .onTrue(new InstantCommand(swerveDrivetrainSubsystem::resetSwerveModulesToAbsolute));
        primaryOperatorController.a().onTrue(new InstantCommand(swerveDrivetrainSubsystem::brake));
        primaryOperatorController.x().whileTrue(new BalanceCommand(swerveDrivetrainSubsystem));

        // Zero location
        secondaryOperatorController
                .a()
                .onTrue(SetPointCommands.stowSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem));

        // Single substation cone
        secondaryOperatorController
                .povDown()
                .onTrue(SetPointCommands.singleSubstationConeSetPoint(
                        elevatorSubsystem, fourbarSubsystem, intakeSubsystem));

        // Single substation cube
        secondaryOperatorController
                .povLeft()
                .onTrue(SetPointCommands.singleSubstationCubeSetPoint(
                        elevatorSubsystem, fourbarSubsystem, intakeSubsystem));

        // Double substation
        secondaryOperatorController
                .povUp()
                .onTrue(SetPointCommands.doubleSubstationSetPoint(
                        elevatorSubsystem, fourbarSubsystem, intakeSubsystem));
        // Ground pickup
        secondaryOperatorController
                .povRight()
                .onTrue(SetPointCommands.groundPickupSePoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem));
        // Mid goal
        secondaryOperatorController
                .x()
                .onTrue(SetPointCommands.midGoalSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem));

        // High goal
        secondaryOperatorController
                .y()
                .onTrue(SetPointCommands.highGoalSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
