package ca.warp7.frc2023.lib.util;

public class SwerveModuleConfig {
    public final int moduleID;
    public final String friendlyName;
    public final int driveMotorID;
    public final int steerMotorID;
    public final int steerEncoderID;
    public final double steerEncoderOffset;

    /*
     * SwerveModule configuration
     */
    public SwerveModuleConfig(
            int moduleID,
            String friendlyName,
            int driveMotorID,
            int steerMotorID,
            int steerEncoderID,
            double steerEncoderOffset) {
        this.friendlyName = friendlyName;
        this.moduleID = moduleID;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.steerEncoderID = steerEncoderID;
        this.steerEncoderOffset = steerEncoderOffset;
    }
}
