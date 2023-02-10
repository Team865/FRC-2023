package ca.warp7.frc2023.subsystems;
import static ca.warp7.frc2023.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fourbar extends SubsystemBase {
    private static Fourbar instance;

    public static Fourbar getInstance() {
        if (instance == null) instance = new Fourbar();
        return instance;
    }

    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    // creates an inverted SparkMAx follower motor, stops death
    public CANSparkMax createFollowerSparkMAX(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(motorMain, true);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // creates a leader and a follower motor
    public CANSparkMax motorMain = createMasterSparkMAX(FOURBAR_MOTOR_PORT_0);
    public CANSparkMax motorSecondary = createFollowerSparkMAX(FOURBAR_MOTOR_PORT_1);

    // the encoder (incremental)
    public RelativeEncoder encoder = motorMain.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); // last value = count per rotation
    
    // Method that creates and sets up a PID controller
    public SparkMaxPIDController createAndSetupPIDController(CANSparkMax motor) {
        SparkMaxPIDController PIDController = motor.getPIDController();
        PIDController.setFeedbackDevice(encoder);
        double kP = 0.1;
        double kI = 1e-4;
        double kD = 1;
        double kIz = 0;
        double kFF = 0;
        double kMaxOutput = 1;
        double kMinOutput = -1;
        int smartMotionSlot = 0;
        double kMaxVel = 2000;
        double kMinVel = 0;
        double kMaxAcc = 1500;
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(kIz);
        PIDController.setFF(kFF);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
        PIDController.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
        PIDController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        PIDController.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
        PIDController.setSmartMotionAllowedClosedLoopError(kMinOutput, smartMotionSlot);
        return PIDController;
    }

    // create and set a PID controller
    public SparkMaxPIDController controllermotorMain = createAndSetupPIDController(motorMain);

    // gets the number of rotations from the encoder
    public double getPosition() {
        return encoder.getPosition();
    }

    // Sets the setPoint of the PID controller to whatever was passed as the input
    public void setPosition(double length) {
        controllermotorMain.setReference(length, CANSparkMax.ControlType.kPosition);
    }
}

//daniel dont beat us
//we tried