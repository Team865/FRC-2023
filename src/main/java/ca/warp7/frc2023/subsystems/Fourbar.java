package ca.warp7.frc2023.subsystems;
import static ca.warp7.frc2023.Constants.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;





public class Fourbar extends SubsystemBase {
    /* fourbar subsytem, used as base to move with PID and remain in requested location
    TO DO:
        - setup PID
        - get PID to automove
        - account for gravity */
    private final CANSparkMax[] motors = { new CANSparkMax(FOURBAR_MOTOR_PORT_0, null), new CANSparkMax(FOURBAR_MOTOR_PORT_1, null) };
    private static Fourbar instance;
    private SparkMaxPIDController PIDController;
    private RelativeEncoder m_encoder;
    private CANSparkMax m_motor;
    
    public static Fourbar getInstance() {
        if (instance == null) instance = new Fourbar();
        return instance;
        
    }

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc;

    private void setMotors(double percent) {
        // motors are connected, both should spin when needed
        motors[0].set(percent);
        motors[1].set(percent);
    
    }
    // Method that creates and sets up a PID controller
    public SparkMaxPIDController createAndSetupPIDController(CANSparkMax motor) {
        m_motor = new CANSparkMax(0, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
        PIDController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
    
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;
    
        // set PID coefficients
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(kIz);
        PIDController.setFF(kFF);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        PIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        PIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        PIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        return PIDController;
    }
    
    // gets the number of rotations from the encoder
    public double getPosition() {
        return m_encoder.getPosition();
    }

       // Sets the setPoint of the PID controller to whatever was passed as the input
    public void setPosition(double length) {
        PIDController.setReference(length, CANSparkMax.ControlType.kPosition);
    }
    
    @Override
    public void periodic() {}


    @Override  
    public void simulationPeriodic() {}
}
