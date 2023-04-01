package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants;
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

        HashMap eventMap = new HashMap<String, Command>();
        //        eventMap.put(
        //                "Cone0",
        //                Commands.sequence(
        //                        intakeSubsystem.setIntakeSpeedCommand(-0.3),
        //                        SetPointCommands.highGoalSetPoint(elevatorSubsystem, fourbarSubsystem,
        // intakeSubsystem),
        //                        new WaitUntilCommand(() -> elevatorSubsystem.isAtPosition()),
        //                        intakeSubsystem.setIntakeSpeedCommand(0.8).withTimeout(1),
        //                        SetPointCommands.stowSetPoint(elevatorSubsystem, fourbarSubsystem, intakeSubsystem),
        //                        intakeSubsystem.setIntakeSpeedCommand(0.0)));
        //        eventMap.put(
        //                "Cone1",
        //                Commands.sequence(
        //                        intakeSubsystem.setIntakeSpeedCommand(-0.3).withTimeout(2),
        //                        SetPointCommands.groundPickupSePoint(elevatorSubsystem, fourbarSubsystem,
        // intakeSubsystem)));
        //        eventMap.put("Stow", SetPointCommands.stowSetPoint(elevatorSubsystem, fourbarSubsystem,
        // intakeSubsystem));

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
