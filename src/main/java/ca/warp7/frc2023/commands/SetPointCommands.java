package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetPointCommands {

    public static Command coneStowSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(elevatorSubsystem
                .setPositionCommand(0)
                .andThen(new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition()))
                .andThen(fourbarSubsystem.setPosition(0, 0.5))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-5)));
    }

    public static Command cubeStowSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(elevatorSubsystem
                .setPositionCommand(0)
                .andThen(new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition()))
                .andThen(fourbarSubsystem.setPosition(0, 0.5))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-5)));
    }

    public static Command singleSubstationConeSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(28, 2.5)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(6))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-5)));
    }

    public static Command singleSubstationCubeSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(57, 2.5)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(2))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-16)));
    }

    public static Command doubleSubstationSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(37, 0.5)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(31))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-17)));
    }

    public static Command groundPickupSePoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(0, 0.5)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(13))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-18)));
    }

    public static Command midGoalSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(48, 15)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(22))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-25)));
    }

    public static Command highGoalSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(56, 15)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(37))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-25)));
    }
}
