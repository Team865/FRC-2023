package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetPointCommands {

    public static Command stowSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(elevatorSubsystem
                .setPositionCommand(0)
                .andThen(new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition()))
                .andThen(fourbarSubsystem.setPosition(0, 0.5))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-5)));
    }

    public static Command singleSubstationSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(fourbarSubsystem
                .setPosition(30, 2.5)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(5)));
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
                .setPosition(57.5, 15)
                .andThen(new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()))
                .andThen(elevatorSubsystem.setPositionCommand(18))
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
