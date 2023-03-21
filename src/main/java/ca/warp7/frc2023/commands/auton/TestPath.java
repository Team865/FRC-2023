package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants.kAuton;
import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.HashMap;
import java.util.List;

public class TestPath implements AutoImpl {
    // private SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final SwerveAutoBuilder autoBuilder;
    private final List<PathPlannerTrajectory> pathGroup;

    public TestPath(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {

        pathGroup = PathPlanner.loadPathGroup(
                "path",
                // new PathConstraints(kAuton.kMaxSpeedMetersPerSecond, kAuton.kMaxAccelerationMetersPerSecondSquared));
                new PathConstraints(2, 1.5));

        HashMap<String, Command> eventMap = new HashMap<>();

        autoBuilder = new SwerveAutoBuilder(
                swerveDrivetrainSubsystem::getPose,
                swerveDrivetrainSubsystem::resetOdometry,
                kDrivetrain.kSwerveDriveKinematics,
                new PIDConstants(kAuton.translationPID.kP, kAuton.translationPID.kI, kAuton.translationPID.kD),
                new PIDConstants(kAuton.rotationPID.kP, kAuton.rotationPID.kI, kAuton.rotationPID.kD),
                swerveDrivetrainSubsystem::setModuleStates,
                eventMap,
                true,
                swerveDrivetrainSubsystem);
    }

    public Pose2d getInitialHolonomicPose() {
        return pathGroup.get(0).getInitialHolonomicPose();
    }

    public Command getCommand() {
        return new SequentialCommandGroup(autoBuilder.fullAuto(pathGroup));
    }
}
