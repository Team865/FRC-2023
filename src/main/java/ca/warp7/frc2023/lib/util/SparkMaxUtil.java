package ca.warp7.frc2023.lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Encoder;

public class SparkMaxUtil {
    // utility methods, saves lines and mental capacity

    // create a master motor, meant to link motors together
    public static CANSparkMax createMasterMotor(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);

        return master;
    }

    // creates an inverted SparkMAx follower motor (links motors together), meant to stop death
    public static CANSparkMax createFollowerMotor(CANSparkMax master, int deviceID, boolean isInverted) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(master, isInverted);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);

        return follower;
    }

    // Method that creates and sets up a PID controller-
    public static SparkMaxPIDController createPIDController(
            CANSparkMax motorGroup,
            Encoder encoder,
            double kP,
            double kI,
            double kD,
            double kMinVel,
            double kMaxVel,
            double kMaxAcc) {

        SparkMaxPIDController PIDController = motorGroup.getPIDController();
        PIDController.setFeedbackDevice((MotorFeedbackSensor) encoder);
        int smartMotionSlot = 0;
        double kMinOutput = -1.0;
        double kMaxOutput = 1.0;

        // kMax and kMin are being used as output range for the kControllerType controller type.
        final CANSparkMax.ControlType kControllerType = CANSparkMax.ControlType.kSmartMotion;

        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(0); // kIz value
        PIDController.setFF(0); // kFF value
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setReference(0, kControllerType);
        PIDController.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
        PIDController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        PIDController.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
        PIDController.setSmartMotionAllowedClosedLoopError(kMinOutput, smartMotionSlot);

        /*  smartdashboard, may need to add specfication if used for other subsytems
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);

        SmartDashboard.putNumber("Max Velocity", kMaxVel);
        SmartDashboard.putNumber("Min Velocity", kMinVel);
        SmartDashboard.putNumber("Max Acceleration", kMaxAcc);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);
        */

        return PIDController;
    }
}
