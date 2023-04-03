package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SetGoalCommands {

    public static Command singleSubstationCone(
            FourbarSubsystem fourbarSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                fourbarSubsystem.setFourbarGoalCommand(28),
                new WaitCommand(0.4)
                        .andThen(elevatorSubsystem.setElevatorGoalCommand(5))
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.SINGLE_SUBSTATION_CONE)),
                intakeSubsystem.setTalonPivotSetPoint(-5));
    }

    public static Command singleSubstationCube(
            FourbarSubsystem fourbarSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                fourbarSubsystem.setFourbarGoalCommand(57),
                new WaitCommand(0.4)
                        .andThen(elevatorSubsystem.setElevatorGoalCommand(5))
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.SINGLE_SUBSTATION_CUBE)),
                intakeSubsystem.setTalonPivotSetPoint(-16));
    }

    public static Command midGoal(
            FourbarSubsystem fourbarSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                fourbarSubsystem.setFourbarGoalCommand(45),
                new WaitCommand(0.5)
                        .andThen(elevatorSubsystem
                                .setElevatorGoalCommand(20.5)
                                .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.MID_GOAL))),
                intakeSubsystem.setTalonPivotSetPoint(-20));
    }

    public static Command highGoal(
            FourbarSubsystem fourbarSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                fourbarSubsystem.setFourbarGoalCommand(52),
                new WaitCommand(0.5)
                        .andThen(elevatorSubsystem
                                .setElevatorGoalCommand(39)
                                .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.HIGH_GOAL))),
                intakeSubsystem.setTalonPivotSetPoint(-22));
    }

    public static Command coneStow(
            FourbarSubsystem fourbarSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem,
            double fourbarWaitSeconds,
            double elevatorWaitSeconds) {
        return Commands.parallel(
                new WaitCommand(fourbarWaitSeconds).andThen(fourbarSubsystem.setFourbarGoalCommand(0)),
                new WaitCommand(elevatorWaitSeconds)
                        .andThen(elevatorSubsystem.setElevatorGoalCommand(0))
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.CONE_STOW)),
                intakeSubsystem.setTalonPivotSetPoint(0));
    }

    public static Command cubeStow(
            FourbarSubsystem fourbarSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem,
            double fourbarWaitSeconds,
            double elevatorWaitSeconds) {
        return Commands.parallel(
                new WaitCommand(fourbarWaitSeconds).andThen(fourbarSubsystem.setFourbarGoalCommand(18)),
                new WaitCommand(elevatorWaitSeconds)
                        .andThen(elevatorSubsystem.setElevatorGoalCommand(0))
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.CUBE_STOW)),
                intakeSubsystem.setTalonPivotSetPoint(0));
    }

    public static Command doubleSubStation(
            FourbarSubsystem fourbarSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                fourbarSubsystem.setFourbarGoalCommand(38),
                elevatorSubsystem
                        .setElevatorGoalCommand(24)
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.DOUBLE_SUBSTATION)),
                intakeSubsystem.setTalonPivotSetPoint(-15));
    }

    public static Command groundPickup(
            FourbarSubsystem fourbarSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem,
            double fourbarWaitSeconds,
            double elevatorWaitSeconds) {
        return Commands.parallel(
                new WaitCommand(fourbarWaitSeconds).andThen(fourbarSubsystem.setFourbarGoalCommand(0)),
                new WaitCommand(elevatorWaitSeconds)
                        .andThen(elevatorSubsystem.setElevatorGoalCommand(13))
                        .andThen(elevatorSubsystem.setCurrentGoal(Constants.Goals.GROUND_PICKUP)),
                intakeSubsystem.setTalonPivotSetPoint(-15));
    }
}
