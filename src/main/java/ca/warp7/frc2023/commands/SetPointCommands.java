package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class SetPointCommands {

    public static Command coneStowSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                elevatorSubsystem.setPositionCommand(0).alongWith(intakeSubsystem.setTalonPivotSetPoint(-2.5)),
                new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition(0.5)),
                fourbarSubsystem.setPosition(0, 0.5));
    }

    public static Command cubeStowSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                elevatorSubsystem.setPositionCommand(0).alongWith(intakeSubsystem.setTalonPivotSetPoint(-2.5)),
                fourbarSubsystem.setPosition(20, 0.5));
    }

    public static Command singleSubstationConeSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                        fourbarSubsystem.setPosition(28, 2.5),
                        new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                        elevatorSubsystem.setPositionCommand(6))
                .alongWith(intakeSubsystem.setTalonPivotSetPoint(-5));
    }

    public static Command singleSubstationCubeSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                fourbarSubsystem.setPosition(57, 2.5),
                new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                elevatorSubsystem.setPositionCommand(2).alongWith(intakeSubsystem.setTalonPivotSetPoint(-16)));
    }

    public static Command doubleSubstationSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                fourbarSubsystem.setPosition(38, 0.5),
                new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                elevatorSubsystem.setPositionCommand(24).alongWith(intakeSubsystem.setTalonPivotSetPoint(-15)));
    }

    public static Command groundPickupSePoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                elevatorSubsystem.setPositionCommand(13),
                new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                fourbarSubsystem.setPosition(0, 0.5).alongWith(intakeSubsystem.setTalonPivotSetPoint(-18)));
    }

    public static Command midGoalSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                fourbarSubsystem.setPosition(50, 10).alongWith(intakeSubsystem.setTalonPivotSetPoint(-25)),
                new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                elevatorSubsystem.setPositionCommand(22));
    }

    public static Command highGoalSetPoint(
            ElevatorSubsystem elevatorSubsystem, FourbarSubsystem fourbarSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.sequence(
                fourbarSubsystem.setPosition(57, 10).alongWith(intakeSubsystem.setTalonPivotSetPoint(-22)),
                new WaitUntilCommand(() -> fourbarSubsystem.isAtPosition()),
                elevatorSubsystem.setPositionCommand(36.5));
    }
}
