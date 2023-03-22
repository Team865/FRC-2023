package ca.warp7.frc2023.lib.util;

import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.lib.config.SwerveModuleConfig;
import ca.warp7.frc2023.lib.math.Conversions;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleUtil {
    /* Module identification */
    public int moduleID;
    public String friendlyName;

    /* Motors */
    private TalonFX driveMotor;
    public TalonFX steerMotor;

    /* Encoders */
    private CANCoder steerEncoder;
    private double steerEncoderOffset;
    // private TalonFXSensorCollection internalSteerEncoder;
    // private TalonFXSensorCollection internalDriveEncoder;

    /* Configuration */
    private CANCoderConfiguration steerEncoderConfig;
    public TalonFXConfiguration driveConfig;
    public TalonFXConfiguration steerConfig;

    /* Other */
    private Rotation2d lastSteerAngle;

    /* Control */
    private SimpleMotorFeedforward driveFeedforward;

    /**
     * @param moduleConfig
     */
    public SwerveModuleUtil(SwerveModuleConfig moduleConfig) {
        this.moduleID = moduleConfig.moduleID;
        this.friendlyName = moduleConfig.friendlyName;

        this.driveFeedforward = new SimpleMotorFeedforward(
                moduleConfig.driveFeedforward.s, moduleConfig.driveFeedforward.v, moduleConfig.driveFeedforward.a);

        this.steerEncoderConfig = moduleConfig.steerEncoderConfig;
        this.driveConfig = moduleConfig.driveConfig;
        this.steerConfig = moduleConfig.steerConfig;
        this.steerEncoderOffset = moduleConfig.steerEncoderOffset;

        /* Create CANCoder */
        this.steerEncoder = new CANCoder(moduleConfig.steerEncoderID);
        configSteerEncoder();

        /* Create steer motor */
        this.steerMotor = new TalonFX(moduleConfig.steerMotorID);
        configSteerMotor();

        /* Create drive motor */
        this.driveMotor = new TalonFX(moduleConfig.driveMotorID);
        configDriveMotor();

        // this.internalSteerEncoder = steerMotor.getSensorCollection();
        // this.internalDriveEncoder = driveMotor.getSensorCollection();

        lastSteerAngle = getState().angle;
    }

    /**
     * Returns angle absolute angle from the module's CANCoder
     * @return
     */
    public Rotation2d getExternalSteerEncodeRotation2d() {
        return Rotation2d.fromDegrees(steerEncoder.getAbsolutePosition());
    }

    /**
     * Resets the steer on the module to point forward
     */
    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(
                getExternalSteerEncodeRotation2d().getDegrees() - steerEncoderOffset, kDrivetrain.kSteerGearRatio);
        steerMotor.setSelectedSensorPosition(absolutePosition);
    }

    /**
     * Returns velocity of the drive motor after conversions
     * @return
     */
    public double getSpeed() {
        return Conversions.falconToMPS(
                driveMotor.getSelectedSensorVelocity(), kDrivetrain.kWheelCircumference, kDrivetrain.kDriveGearRatio);
    }

    public Rotation2d getInternalSteerEncodeRotation2d() {
        return Rotation2d.fromDegrees(
                Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), kDrivetrain.kSteerGearRatio));
    }

    /**
     * Returns state of the module
     * @return
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeed(), getInternalSteerEncodeRotation2d());
    }

    /**
     * Returns total displacement of the drive motor after conversions
     * @return
     */
    public double getDistance() {
        return Conversions.falconToMeters(
                driveMotor.getSelectedSensorPosition(), kDrivetrain.kWheelCircumference, kDrivetrain.kDriveGearRatio);
    }

    /**
     * Returns position of the module
     *
     * @return SwerveModulePosition of module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), getInternalSteerEncodeRotation2d());
    }

    /**
     * Set the angle of the steer motor on the swerve module
     * @param desiredState
     */
    private void setAngle(SwerveModuleState desiredState) {

        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (kDrivetrain.kMaxSpeed * 0.05))
                ? lastSteerAngle
                : desiredState.angle;

        steerMotor.set(
                TalonFXControlMode.Position,
                Conversions.degreesToFalcon(angle.getDegrees(), kDrivetrain.kSteerGearRatio));
        lastSteerAngle = angle;
    }

    /**
     * Set the angle of the steer motor on the swerve module without checking if the speed is low
     * @param desiredState
     */
    public void setAngleNoCheck(SwerveModuleState desiredState) {
        steerMotor.set(
                TalonFXControlMode.Position,
                Conversions.degreesToFalcon(desiredState.angle.getDegrees(), kDrivetrain.kSteerGearRatio));
    }



    /**
     * Set the speed of the drive motor on the swerve module
     * @param desiredState
     * @param isOpenLoop
     */
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kDrivetrain.kMaxSpeed;
            driveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(
                    desiredState.speedMetersPerSecond, kDrivetrain.kWheelCircumference, kDrivetrain.kDriveGearRatio);
            driveMotor.set(
                    TalonFXControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    driveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        // Optimize the state to avoid turning > 90 degrees.
        desiredState = ContinuousModuleStateUtil.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the desired state for the module.
     * @param desiredState
     */
    public void setDesiredStateNoCheck(SwerveModuleState desiredState, boolean isOpenLoop) {

        // Optimize the state to avoid turning > 90 degrees.
        desiredState = ContinuousModuleStateUtil.optimize(desiredState, getState().angle);

        setAngleNoCheck(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }


    /**
     * Configures CANCoder
     */
    private void configSteerEncoder() {
        steerEncoder.configFactoryDefault(250);
        steerEncoder.configAllSettings(steerEncoderConfig, 250);
        steerEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);
    }

    /**
     * Configures drive motors
     */
    private void configDriveMotor() {
        driveMotor.configFactoryDefault(250);
        driveMotor.configAllSettings(driveConfig, 250);
        driveMotor.setInverted(false);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * Configures steer motors
     */
    private void configSteerMotor() {
        steerMotor.configFactoryDefault(250);
        steerMotor.configAllSettings(steerConfig, 250);
        steerMotor.setInverted(true);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        steerMotor.enableVoltageCompensation(true);
        steerMotor.setStatusFramePeriod(1, 100);
    }
}
