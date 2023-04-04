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

public class Mobility1andHalfConeBalance extends SequentialCommandGroup {
    public Mobility1andHalfConeBalance(
            ElevatorSubsystem elevatorSubsystem,
            FourbarSubsystem fourbarSubsystem,
            IntakeSubsystem intakeSubsystem,
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {

        List<PathPlannerTrajectory> examplePath = PathPlanner.loadPathGroup(
                "1.5", new PathConstraints(2, 0.5), new PathConstraints(1.5, 1), new PathConstraints(6.75, 3));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put(
                "0ScoreHigh",
                Commands.sequence(
                        intakeSubsystem.setIntakeSpeedCommand(-0.3),
                        SetGoalCommands.highGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem),
                        intakeSubsystem.setIntakeSpeedCommand(0.5).withTimeout(1),
                        SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0.4, 0),
                        intakeSubsystem.setIntakeSpeedCommand(0.0)));
        eventMap.put(
                "1GroundIntakeCone",
                Commands.sequence(
                        intakeSubsystem.setIntakeSpeedCommand(-0.8).withTimeout(0.5),
                        intakeSubsystem.setIntakeSpeedCommand(-0.3),
                        SetGoalCommands.groundPickup(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0)));
        eventMap.put("2Stow", SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0));
        eventMap.put("3AutoBalance", new BalanceCommand(swerveDrivetrainSubsystem, false).withTimeout(3));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerveDrivetrainSubsystem::getPose,
                swerveDrivetrainSubsystem::resetOdometry,
                Constants.kDrivetrain.kSwerveDriveKinematics,
                new PIDConstants(0.1, 0.0, 0.0),
                new PIDConstants(0.75, 0.0, 0.0),
                swerveDrivetrainSubsystem::setModuleStates,
                eventMap,
                false,
                swerveDrivetrainSubsystem);

        addCommands(
                // Zero swerve
                new InstantCommand(() -> swerveDrivetrainSubsystem.resetOdometry(
                        examplePath.get(0).getInitialHolonomicPose())),
                autoBuilder.fullAuto(examplePath));
    }
}
