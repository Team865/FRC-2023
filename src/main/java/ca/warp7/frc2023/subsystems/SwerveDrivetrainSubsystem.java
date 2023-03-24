package ca.warp7.frc2023.subsystems;

import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.lib.util.SwerveModuleUtil;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    public SwerveModuleUtil[] swerveModules;
    public SwerveDriveOdometry swerveDriveOdometry;
    public AHRS navX;
    public boolean isBrakeEnabled = false;

    public SwerveDrivetrainSubsystem() {
        navX = new AHRS(SPI.Port.kMXP);
        zeroGyro();

        // Creates the swerve modules, Each module is assigned an ID
        swerveModules = new SwerveModuleUtil[] {
            new SwerveModuleUtil(Constants.kDrivetrain.kModule0.kConfig),
            new SwerveModuleUtil(Constants.kDrivetrain.kModule1.kConfig),
            new SwerveModuleUtil(Constants.kDrivetrain.kModule2.kConfig),
            new SwerveModuleUtil(Constants.kDrivetrain.kModule3.kConfig)
        };

        // Pause to allow for CANCoder to initialize. Avoids pulling bad data and not aligning
        Timer.delay(0.5);
        resetSwerveModulesToAbsolute();

        swerveDriveOdometry = new SwerveDriveOdometry(
                Constants.kDrivetrain.kSwerveDriveKinematics, getYawRotation2d(), getSwerveModulePositions());
    }

    /**
     * Takes parameters and calculates kinematics to apply to swerve modules
     *
     * @param translation
     * @param rotation
     * @param isFieldOriented
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean isFieldOriented, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.kDrivetrain.kSwerveDriveKinematics.toSwerveModuleStates(
                isFieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYawRotation2d())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kDrivetrain.kMaxSpeed);

        for (SwerveModuleUtil module : swerveModules) {
            module.setDesiredState(swerveModuleStates[module.moduleID], isOpenLoop);
        }
    }

    /**
     * Gets the pose of the robot
     *
     * @return Pose2d of the robot
     */
    public Pose2d getPose() {
        return swerveDriveOdometry.getPoseMeters();
    }

    /**
     * Reset the odometry to Pose2d
     *
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        swerveDriveOdometry.resetPosition(getYawRotation2d(), getSwerveModulePositions(), pose);
    }

    /**
     * Gets the state of swerve modules
     *
     * @return the state of swerve modules in an array
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModuleUtil module : swerveModules) {
            states[module.moduleID] = module.getState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (SwerveModuleUtil module : swerveModules) {
            module.setDesiredState(states[module.moduleID], true);
        }
    }

    /**
     * Gets the positions of positions of swerve modules
     *
     * @return the positions of swerve modules in an array
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModuleUtil module : swerveModules) {
            positions[module.moduleID] = module.getPosition();
        }
        return positions;
    }

    /**
     * Zeroes the navX
     */
    public void zeroGyro() {
        navX.zeroYaw();
    }

    /**
     * Uses roll from the navX since it is mounted at 90 degrees
     *
     * @return the pitch of the robot
     */
    public double getPitch() {
        return navX.getRoll(); // Uses role since navX is 90 deg.
    }

    /**
     * Get yaw of the robot from navX as rotation 2d
     * Uses fused heading if navX is calibrated
     *
     * @return Rotation2d of yaw
     */
    public Rotation2d getYawRotation2d() {
        // Don't use fused heading even if available for. Fix later TODO
        return false ? Rotation2d.fromDegrees(navX.getFusedHeading()) : Rotation2d.fromDegrees(360 - navX.getYaw());
    }

    /**
     * Reset swerve modules to absolute
     */
    public void resetSwerveModulesToAbsolute() {
        for (SwerveModuleUtil module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    private void enableBrake() {
        SwerveModuleUtil[] swerveModules = this.swerveModules;
        swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true, false);
        swerveModules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true, false);
        swerveModules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true, false);
        swerveModules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true, false);
        isBrakeEnabled = true;
    }

    private void disableBrake() {
        SwerveModuleUtil[] swerveModules = this.swerveModules;
        swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true, false);
        swerveModules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true, false);
        swerveModules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true, false);
        swerveModules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true, false);
        isBrakeEnabled = false;
    }

    public boolean isBrakeEnabled() {
        return isBrakeEnabled;
    }

    // TODO: No idea how this works yet lol
    // public SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
    //         Constants.kDrivetrain.kSwerveDriveKinematics,
    //         getYawRotation2d(),
    //         getSwerveModulePositions(),
    //         new Pose2d(),
    //         VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
    //         VecBuilder.fill(0.5, 0.5, Units.degr333333333333333333333333333333333333eesToRadians(30)));

    public Command zeroCommand() {
        return runOnce(() -> this.zeroGyro());
    }

    public Command brakeCommand() {
        return startEnd(() -> enableBrake(), () -> disableBrake());
    }

    public Command mobilty() {
        return run(() -> this.drive(new Translation2d(-1.0, 0.0).times(0.8), 0.0, false, true));
    }

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getPose, // Pose supplier
                        Constants.kDrivetrain.kSwerveDriveKinematics, // SwerveDriveKinematics
                        new PIDController(
                                0.1, 0,
                                0), // X controller. Tune these values for your robot. Leaving them 0 will only use
                        // feedforwards.
                        new PIDController(0.1, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(
                                0.75, 0,
                                0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only
                        // use feedforwards.
                        this::setModuleStates, // Module states consumer
                        false, // Should the path be automatically mirrored depending on alliance color. Optional,
                        // defaults to true
                        this // Requires this drive subsystem
                        ));
    }

    /**
     * Looped subsystem code
     */
    @Override
    public void periodic() {
        // Constantly update module positions
        swerveDriveOdometry.update(getYawRotation2d(), getSwerveModulePositions());

        SmartDashboard.putBoolean("Swerve drive brake", isBrakeEnabled);

        SmartDashboard.putNumber("Robot X", swerveDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Robot Y", swerveDriveOdometry.getPoseMeters().getY());

        SmartDashboard.putBoolean("Is Magnetometer Calibrated", navX.isMagnetometerCalibrated());
        SmartDashboard.putNumber("NavX rotation", getYawRotation2d().getDegrees());
    }
}
