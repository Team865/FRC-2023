package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {
        // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max
        // acceleration of 3 m/s^2
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("TestAuto", new PathConstraints(1, 0.5));

        addCommands(swerveDrivetrainSubsystem.followTrajectoryCommand(examplePath, true));
    }
}
