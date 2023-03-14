package ca.warp7.frc2023.commands.auton;

import java.util.HashMap;
import java.util.List;

import ca.warp7.frc2023.Constants.kAuton;
import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class Cone implements AutoImpl{
    // private final PoseEstimator poseEstimator;
    // private final Swerve swerve;
    // private final Arm arm;
    // private final Wrist wrist;
    // private final LEDs leds;
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final SwerveAutoBuilder autoBuilder;
    private final List<PathPlannerTrajectory> pathGroup;

    public Cone(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem, IntakeSubsystem intakeSubsystem) {
        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        pathGroup = PathPlanner.loadPathGroup("Cone", 
            new PathConstraints(kAuton.kMaxAccelerationMetersPerSecondSquared, 
                                kAuton.kMaxAccelerationMetersPerSecondSquared)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        
        autoBuilder = new SwerveAutoBuilder(
            poseEstimator::getCurrentPose,
            poseEstimator::setCurrentPose,
            kDrivetrain.kSwerveDriveKinematics,
            new PIDConstants(kAuton.translationPID.kP, kAuton.translationPID.kI, kAuton.translationPID.kD),
            new PIDConstants(kAuton.rotationPID.kP, kAuton.rotationPID.kI, kAuton.rotationPID.kD),
            swerveDrivetrainSubsystem::setModuleStates, eventMap, true, swerveDrivetrainSubsystem);
    }

    public Pose2d getInitialHolonomicPose() {
        return pathGroup.get(0).getInitialHolonomicPose();
      }
    
    public Command getCommand() {
        return new SequentialCommandGroup(
            intakeSubsystem.setIntakeSpeedCommand(kAuton.kOuttakeSpeed),
            Commands.waitSeconds(1)
            // autoBuilder.fullAuto(pathGroup), new Balance(swerve, leds))
        );
      }
}
