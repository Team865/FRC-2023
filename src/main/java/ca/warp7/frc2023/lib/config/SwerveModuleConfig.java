package ca.warp7.frc2023.lib.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModuleConfig {
    public final int moduleID;
    public final String friendlyName;
    public final int driveMotorID;
    public final int steerMotorID;
    public final int steerEncoderID;
    public final double steerEncoderOffset;
    public final CANCoderConfiguration steerEncoderConfig;
    public final TalonFXConfiguration driveConfig;
    public final TalonFXConfiguration steerConfig;
    public final SVAConfig driveFeedforward;

    /**
     * Swerve module configuration
     *
     * @param moduleID
     * @param friendlyName
     * @param driveMotorID
     * @param steerMotorID
     * @param steerEncoderID
     * @param steerEncoderOffset
     * @param steerEncoderConfig
     * @param driveConfig
     * @param steerConfig
     */
    public SwerveModuleConfig(
            int moduleID,
            String friendlyName,
            int driveMotorID,
            int steerMotorID,
            int steerEncoderID,
            double steerEncoderOffset,
            CANCoderConfiguration steerEncoderConfig,
            TalonFXConfiguration driveConfig,
            TalonFXConfiguration steerConfig,
            SVAConfig driveFeedforward) {
        this.friendlyName = friendlyName;
        this.moduleID = moduleID;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.steerEncoderID = steerEncoderID;
        this.steerEncoderOffset = steerEncoderOffset;
        this.steerEncoderConfig = steerEncoderConfig;
        this.driveConfig = driveConfig;
        this.steerConfig = steerConfig;
        this.driveFeedforward = driveFeedforward;
    }
}
