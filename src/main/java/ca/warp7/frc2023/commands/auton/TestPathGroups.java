package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.*;
import edu.wpi.first.wpilibj2.command.*;
import java.util.HashMap;
import java.util.List;

public class TestPathGroups extends SequentialCommandGroup {
    public TestPathGroups(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {

        List<PathPlannerTrajectory> examplePath =
                PathPlanner.loadPathGroup("TestPathGroups", new PathConstraints(2, 0.5), new PathConstraints(3.5, 1));

        HashMap eventMap = new HashMap<String, Command>();
        eventMap.put("test", new PrintCommand("event good"));

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
