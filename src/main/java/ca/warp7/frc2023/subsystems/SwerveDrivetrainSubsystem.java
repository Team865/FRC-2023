package ca.warp7.frc2023.subsystems;

import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.lib.util.SwerveModuleUtil;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrainSubsystem extends SubsystemBase {
    public SwerveModuleUtil[] swerveModules;
    public SwerveDriveOdometry swerveDriveOdometry;
    public AHRS navX;

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
        Timer.delay(1);
        resetSwerveModulesToAbsolute();

        swerveDriveOdometry = new SwerveDriveOdometry(
                Constants.kDrivetrain.kSwerveDriveKinematics, getYawRotation2d(), getSwerveModulePositions());

            // TODO: No idea how this works yet lol
        swerveDrivePoseEstimator_warp = new SwerveDrivePoseEstimator(
        kDrivetrain.kSwerveDriveKinematics,
        getYawRotation2d(),
        getSwerveModulePositions(),
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
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

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kDrivetrain.kMaxSpeed);

        for (SwerveModuleUtil module : swerveModules) {
            module.setDesiredState(desiredStates[module.moduleID], false);
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
        this.navX.zeroYaw();
    }

    /**
     * Get yaw of the robot from navX as rotation 2d
     * Uses fused heading if navX is calibrated
     *
     * @return Rotation2d of yaw
     */
    public Rotation2d getYawRotation2d() {
        // Don't use fused heading even if available for. Fix later TODO
        System.out.println(this.navX);
        return false ? Rotation2d.fromDegrees(navX.getFusedHeading()) : Rotation2d.fromDegrees(360 - this.navX.getYaw());
    }

    /**
     * Reset swerve modules to absolute
     */
    public void resetSwerveModulesToAbsolute() {
        for (SwerveModuleUtil module : swerveModules) {
            module.resetToAbsolute();
        }
    }

    public void brake() {
        SwerveModuleUtil[] swerveModules = this.swerveModules;
        swerveModules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        swerveModules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true);
        swerveModules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(315)), true);
        swerveModules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
    }



    //https://github.com/FRC3161/ChargedUp2023/blob/main/src/main/java/frc/robot/subsystems/PoseEstimator.java
    public void setCurrentPose(Pose2d newPose) {
        swerveDrivePoseEstimator_warp.resetPosition(getYawRotation2d(), getSwerveModulePositions(), newPose);
    }

    public Command mobilty() {
        return run(() -> this.drive(new Translation2d(-1.0, 0.0).times(0.8), 0.0, false, true));
    }

    /**
     * Looped subsystem code
     */
    @Override
    public void periodic() {
        // Constantly update module positions
        swerveDriveOdometry.update(getYawRotation2d(), getSwerveModulePositions());

        SmartDashboard.putBoolean("Is Magnetometer Calibrated", navX.isMagnetometerCalibrated());
        SmartDashboard.putNumber("NavX rotation", getYawRotation2d().getDegrees());
    }
}
