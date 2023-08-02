package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.*;
import java.util.List;

public class WallHighGround extends SequentialCommandGroup {
    public WallHighGround(
            ElevatorSubsystem elevatorSubsystem,
            FourbarSubsystem fourbarSubsystem,
            IntakeSubsystem intakeSubsystem,
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {

        List<PathPlannerTrajectory> wall_high_ground__1 = PathPlanner.loadPathGroup(
                "wall_high_ground-1",
                new PathConstraints(2, 2),
                new PathConstraints(1, 0.5),
                new PathConstraints(1, 0.5),
                new PathConstraints(1, 0.5),
                new PathConstraints(2, 2),
                new PathConstraints(0.5, 0.5));

        ConfiguredSwerveAutoBuilder swerveAutoBuilder = new ConfiguredSwerveAutoBuilder(
                swerveDrivetrainSubsystem, fourbarSubsystem, elevatorSubsystem, intakeSubsystem);

        addCommands(
                // Zero swerve
                new InstantCommand(() -> {
                    swerveDrivetrainSubsystem.resetOdometry(
                            wall_high_ground__1.get(0).getInitialHolonomicPose());
                    swerveAutoBuilder.getConfiguredAutoBuilder().fullAuto(wall_high_ground__1);
                }));
    }
}
