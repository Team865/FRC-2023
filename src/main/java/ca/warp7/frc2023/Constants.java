package ca.warp7.frc2023;

import ca.warp7.frc2023.lib.config.ProfiledPIDConfig;
import ca.warp7.frc2023.lib.config.SVAConfig;
import ca.warp7.frc2023.lib.config.SwerveModuleConfig;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double kBatteryNominalVoltage = 12.0;

    public enum Goals {
        NONE,
        CONE_STOW,
        CUBE_STOW,
        MID_GOAL,
        HIGH_GOAL,
        GROUND_PICKUP,
        SINGLE_SUBSTATION_CONE,
        SINGLE_SUBSTATION_CUBE,
        DOUBLE_SUBSTATION
    }

    /* Drivetrain Information */
    public static final class kDrivetrain {
        public static final double kTrackWidth = Units.inchesToMeters(30.75);
        public static final double kWheelBase = Units.inchesToMeters(18.75);
        public static final double kWheelDiameter = Units.inchesToMeters(4);
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kMaxSpeed = Units.feetToMeters(16.3);
        // This should calculate the theoretical max angular velocity using the max speed and the longer radius of the
        // robot
        public static final double kMaxAngularVelocity = kMaxSpeed / (kTrackWidth / 2);
        public static final double kDriveGearRatio = 6.75 / 1;
        public static final double kSteerGearRatio = (150.0 / 7.0) / 1.0;

        public static final SVAConfig kDriveFeedforward = new SVAConfig(0.21964, 2.1732, 0.5249);

        public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kDrivetrain.kTrackWidth / 2.0, kDrivetrain.kWheelBase / 2.0),
                new Translation2d(kDrivetrain.kTrackWidth / 2.0, -kDrivetrain.kWheelBase / 2.0),
                new Translation2d(-kDrivetrain.kTrackWidth / 2.0, kDrivetrain.kWheelBase / 2.0),
                new Translation2d(-kDrivetrain.kTrackWidth / 2.0, -kDrivetrain.kWheelBase / 2.0));

        private static final CANCoderConfiguration kSteerEncoderConfig = new CANCoderConfiguration() {
            {
                absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
                sensorDirection = false;
                initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
                sensorTimeBase = SensorTimeBase.PerSecond;
            }
        };

        private static final SupplyCurrentLimitConfiguration kDriveSupplyLimit =
                new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);

        private static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration() {
            {
                slot0.kP = 0;
                slot0.kI = 0;
                slot0.kD = 0;
                slot0.kF = 0;
                supplyCurrLimit = kDriveSupplyLimit;
                voltageCompSaturation = 12.0;
            }
        };

        private static final SupplyCurrentLimitConfiguration kAngleSupplyLimit =
                new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);

        private static final TalonFXConfiguration kSteerConfig = new TalonFXConfiguration() {
            {
                slot0.kP = 0.25;
                slot0.kI = 0.0;
                slot0.kD = 0.05;
                supplyCurrLimit = kAngleSupplyLimit;
                voltageCompSaturation = 12.0;
            }
        };

        public static final class kModule0 {
            private static final int kModuleID = 0;
            private static final String kFriendlyName = "Front left";
            private static final int kDriveMotorID = 22;
            private static final int kSteerMotorID = 21;
            private static final int kSteerEncoderID = 20;
            private static final double kSteerEncoderOffset = 4.86;

            public static final SwerveModuleConfig kConfig = new SwerveModuleConfig(
                    kModuleID,
                    kFriendlyName,
                    kDriveMotorID,
                    kSteerMotorID,
                    kSteerEncoderID,
                    kSteerEncoderOffset,
                    kSteerEncoderConfig,
                    kDriveConfig,
                    kSteerConfig,
                    kDriveFeedforward);
        }

        public static final class kModule1 {
            private static final int kModuleID = 1;
            private static final String kFriendlyName = "Front right";
            private static final int kDriveMotorID = 32;
            private static final int kSteerMotorID = 31;
            private static final int kSteerEncoderID = 30;
            private static final double kSteerEncoderOffset = 141;

            public static final SwerveModuleConfig kConfig = new SwerveModuleConfig(
                    kModuleID,
                    kFriendlyName,
                    kDriveMotorID,
                    kSteerMotorID,
                    kSteerEncoderID,
                    kSteerEncoderOffset,
                    kSteerEncoderConfig,
                    kDriveConfig,
                    kSteerConfig,
                    kDriveFeedforward);
        }

        public static final class kModule2 {
            private static final int kModuleID = 2;
            private static final String kFriendlyName = "Back left";
            private static final int kDriveMotorID = 42;
            private static final int kSteerMotorID = 41;
            private static final int kSteerEncoderID = 40;
            private static final double kSteerEncoderOffset = 52.91;

            public static final SwerveModuleConfig kConfig = new SwerveModuleConfig(
                    kModuleID,
                    kFriendlyName,
                    kDriveMotorID,
                    kSteerMotorID,
                    kSteerEncoderID,
                    kSteerEncoderOffset,
                    kSteerEncoderConfig,
                    kDriveConfig,
                    kSteerConfig,
                    kDriveFeedforward);
        }

        public static final class kModule3 {
            private static final int kModuleID = 3;
            private static final String kFriendlyName = "Back right";
            private static final int kDriveMotorID = 52;
            private static final int kSteerMotorID = 51;
            private static final int kSteerEncoderID = 50;
            private static final double kSteerEncoderOffset = 283.5;

            public static final SwerveModuleConfig kConfig = new SwerveModuleConfig(
                    kModuleID,
                    kFriendlyName,
                    kDriveMotorID,
                    kSteerMotorID,
                    kSteerEncoderID,
                    kSteerEncoderOffset,
                    kSteerEncoderConfig,
                    kDriveConfig,
                    kSteerConfig,
                    kDriveFeedforward);
        }
    }

    public static final class kTeleop {
        public static final double kDriveDeadband = 0.05;
        public static final double kRotateDeadband = 0.75;
        public static final double kElevatorStickDeadband = 0.10;
        public static final double kFourbarStickDeadband = 0.25;
        public static final double kDriveSingleDirectionPercent = 0.2;
        public static final double kTriggerDeadband = 0.05;
    }

    public static final class kAuton {
        public static final PIDConstants kTranslationalController = new PIDConstants(0.5, 0, 0);
        public static final PIDConstants kRotationalController = new PIDConstants(4, 0, 0);
    }

    /* Controller USB Order */
    public static final class kControllers {
        public static final int kDriverControllerID = 0;
        public static final int kOperatorControllerID = 1;
        public static final int kTechnicianControllerID = 2;
    }

    public static final class kFourbar {
        public static final int kMotorLeftID = 13;
        public static final int kMotorRightID = 10;

        public static final double kConeCubeStowFromScoreWait = 0.4;
    }

    public static final class kElevator {
        public static final int kMotorGroupID = 9;

        public static final int kLimitSwitchID = 2;

        public static final int[] kEncoderIDs = {0, 1};
        public static final boolean kEncoderReversed = false;

        public static final double kEncoderDistancePerPulse = 1 / (24.38 * 2 * Math.PI * 0.8755);

        public static final ProfiledPIDConfig kProfiledPIDController = new ProfiledPIDConfig(1.8, 0, 0, 90, 250, 0.1);

        public static final double kCubeStowFromGround = 0.75;
        public static final double kGroundFromCubeStow = 0.5;
    }

    public static final class kIntake {
        public static final double kOuttakeSpeed = 0.5;
        public static final double kFeedforwardSpeed = -0.3;
        public static final int kFrontWheelMotorID = 11;
        public static final int kRearWheelMotorID = 18;
        public static final int kTalonPivotMotorID = 12;
    }
}
