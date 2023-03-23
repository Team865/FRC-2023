package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.commands.BalanceCommand;
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

public class MobilityConeBalance extends SequentialCommandGroup {
    public MobilityConeBalance(
            ElevatorSubsystem elevatorSubsystem,
            FourbarSubsystem fourbarSubsystem,
            IntakeSubsystem intakeSubsystem,
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max
        // acceleration of 3 m/s^2
        PathConstraints constraints = PathPlanner.getConstraintsFromPath("MobilityCone Copy");
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("MobilityCone Copy", constraints);

        addCommands(
                intakeSubsystem.setIntakeSpeedCommand(-0.3),
                SetPointCommands.highGoalSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem),
                new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition()),
                intakeSubsystem.setIntakeSpeedCommand(0.8).withTimeout(1),
                SetPointCommands.stowSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem),
                intakeSubsystem.setIntakeSpeedCommand(0.0),
                swerveDrivetrainSubsystem.followTrajectoryCommand(examplePath, true),
                new BalanceCommand(swerveDrivetrainSubsystem));
    }
}
