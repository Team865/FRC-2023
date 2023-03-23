package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.commands.SetPointCommands;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class MobilityCone extends SequentialCommandGroup {
    public MobilityCone(
            ElevatorSubsystem elevatorSubsystem,
            FourbarSubsystem fourbarSubsystem,
            IntakeSubsystem intakeSubsystem,
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max
        // acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("MobilityCone", new PathConstraints(4.0, 1.0));

        addCommands(
                SetPointCommands.highGoalSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem),
                new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition())
                        .alongWith(intakeSubsystem.setIntakeSpeedCommand(-0.3)),
                intakeSubsystem.setIntakeSpeedCommand(0.8).withTimeout(1),
                SetPointCommands.coneStowSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem),
                swerveDrivetrainSubsystem.followTrajectoryCommand(examplePath, true));
    }
}
