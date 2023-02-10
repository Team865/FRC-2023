package ca.warp7.frc2023.lib;
import ca.warp7.frc2023.lib.SparkMaxUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    // Method that creates and sets up a PID controller
    public static SparkMaxPIDController createPIDController(CANSparkMax motorGroup, RelativeEncoder motorEncoder, 
            double kP, double kI, double kD, double kMinVel, double kMaxVel, double kMaxAcc) {

        SparkMaxPIDController PIDController = motorGroup.getPIDController();
        PIDController.setFeedbackDevice(motorEncoder);
        int smartMotionSlot = 0;
        double kMaxOutput = 1;
        double kMinOutput = -1;

        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(0); // kIz value
        PIDController.setFF(0); // kFF value
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        PIDController.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
        PIDController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        PIDController.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
        PIDController.setSmartMotionAllowedClosedLoopError(kMinOutput, smartMotionSlot);

        // smartdashboard, may need to add specfication if used for other subsytems
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);

        return PIDController;
    }
    //We are sorry daniel
    //dont kill us
}
