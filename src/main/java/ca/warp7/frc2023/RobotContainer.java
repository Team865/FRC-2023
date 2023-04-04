// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import ca.warp7.frc2023.Constants.kControllers;
import ca.warp7.frc2023.commands.*;
import ca.warp7.frc2023.commands.auton.*;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
                () -> primaryOperatorController.rightBumper().getAsBoolean(),
                () -> primaryOperatorController.getHID().getPOV()));

        intakeSubsystem.setDefaultCommand(new TeleopIntakeCommand(
                intakeSubsystem,
                () -> secondaryOperatorController.getRightTriggerAxis(),
                () -> MathUtil.clamp(secondaryOperatorController.getLeftTriggerAxis(), 0, 0.8),
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
        autoChooser.addOption(
                "Cone and mobility",
                new MobilityCone(elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "MobilityConeBalance",
                new MobilityConeBalance(
                        elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption("TestGroups", new TestPathGroups(swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "1.5",
                new Mobility1andHalfConeBalance(
                        elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "HighBalance",
                new HighBalance(elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "1.5Bump",
                new OneAndHalfBump(elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "2High", new TwoHigh(elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        autoChooser.addOption(
                "TestAuto",
                new TestAuto(elevatorSubsystem, fourbarSubsystem, intakeSubsystem, swerveDrivetrainSubsystem));
        SmartDashboard.putData("autoChooser", autoChooser);
    }

    private void configureBindings() {
        /*
         * Driver
         */
        primaryOperatorController.back().onTrue(swerveDrivetrainSubsystem.zeroGyroCommand());
        primaryOperatorController
                .start()
                .onTrue(new InstantCommand(swerveDrivetrainSubsystem::resetSwerveModulesToAbsolute));
        // Toggle brake
        primaryOperatorController.b().toggleOnTrue(swerveDrivetrainSubsystem.brakeCommand());
        // Balance
        primaryOperatorController.x().whileTrue(new BalanceCommand(swerveDrivetrainSubsystem, true));

        /*
         * Operator
         */
        // Cone stow set point
        secondaryOperatorController
                .a()
                .and(secondaryOperatorController.povDown())
                .onTrue(new ConditionalCommand(
                        SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0.4, 0),
                        SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0),
                        () -> elevatorSubsystem.currentGoal == Constants.Goals.HIGH_GOAL
                                || elevatorSubsystem.currentGoal == Constants.Goals.MID_GOAL));
        // Cube stow set point
        secondaryOperatorController
                .a()
                .and(secondaryOperatorController.povLeft())
                .onTrue(new ConditionalCommand(
                        SetGoalCommands.cubeStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0.4, 0),
                        SetGoalCommands.cubeStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0.75),
                        () -> elevatorSubsystem.currentGoal == Constants.Goals.HIGH_GOAL
                                || elevatorSubsystem.currentGoal == Constants.Goals.MID_GOAL));

        // Single substation cone set point
        secondaryOperatorController
                .x()
                .and(secondaryOperatorController.povDown())
                .onTrue(SetGoalCommands.singleSubstationCone(fourbarSubsystem, elevatorSubsystem, intakeSubsystem));

        // Single substation cube set point
        secondaryOperatorController
                .x()
                .and(secondaryOperatorController.povLeft())
                .onTrue(SetGoalCommands.singleSubstationCube(fourbarSubsystem, elevatorSubsystem, intakeSubsystem));

        // Ground pickup
        secondaryOperatorController
                .a()
                .and(secondaryOperatorController.povRight())
                .onTrue(new ConditionalCommand(
                        SetGoalCommands.groundPickup(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0.5),
                        SetGoalCommands.groundPickup(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0),
                        () -> elevatorSubsystem.currentGoal == Constants.Goals.CUBE_STOW));
        // Mid goal
        secondaryOperatorController
                .b()
                .and(secondaryOperatorController.povLeft())
                .onTrue(SetGoalCommands.midGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem));

        // High goal
        secondaryOperatorController
                .b()
                .and(secondaryOperatorController.povUp())
                .onTrue(SetGoalCommands.highGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem));

        // Double substation
        secondaryOperatorController
                .y()
                .and(secondaryOperatorController.povUp())
                .onTrue(SetGoalCommands.doubleSubStation(fourbarSubsystem, elevatorSubsystem, intakeSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
