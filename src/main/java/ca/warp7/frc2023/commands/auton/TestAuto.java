package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.commands.BalanceCommand;
import ca.warp7.frc2023.commands.SetGoalCommands;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.*;
import java.util.HashMap;
import java.util.List;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(
            ElevatorSubsystem elevatorSubsystem,
            FourbarSubsystem fourbarSubsystem,
            IntakeSubsystem intakeSubsystem,
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(
                "TestAuto", new PathConstraints(2, 2), new PathConstraints(1, 0.5), new PathConstraints(1, 0.5));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put(
                "ScoreHighCone",
                Commands.sequence(
                        intakeSubsystem.setIntakeSpeedCommand(-0.3),
                        SetGoalCommands.highGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem),
                        new WaitCommand(1.25),
                        intakeSubsystem.setIntakeSpeedCommand(0.5),
                        new WaitCommand(0.1),
                        intakeSubsystem.setIntakeSpeedCommand(0.0),
                        SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0.5, 0),
                        new WaitCommand(1)));

        eventMap.put("AutoBalance", new BalanceCommand(swerveDrivetrainSubsystem, false).withTimeout(5));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerveDrivetrainSubsystem::getPose,
                swerveDrivetrainSubsystem::resetOdometry,
                Constants.kDrivetrain.kSwerveDriveKinematics,
                new PIDConstants(0.3, 0.0, 0.0),
                new PIDConstants(4, 0.0, 0.0),
                swerveDrivetrainSubsystem::setModuleStates,
                eventMap,
                false,
                swerveDrivetrainSubsystem);

        addCommands(
                // Zero swerve
                new InstantCommand(() ->
                        swerveDrivetrainSubsystem.resetOdometry(path.get(0).getInitialHolonomicPose())),
                autoBuilder.fullAuto(path));
    }
}
