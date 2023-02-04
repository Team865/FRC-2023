package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    // returns the instance of the subsystem
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    public CANSparkMax createFollowerSparkMAX(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(LeftMain);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // creates an inverted SparkMAx follower motor
    public CANSparkMax createFollowerSparkMAXInverted(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(LeftMain, true);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // makes a leader and a follower motor for the left side
    public CANSparkMax LeftMain = createMasterSparkMAX(kElevatorLeftMain);
    public CANSparkMax LeftSecondary = createFollowerSparkMAX(kElevatorLeftSecondary);

    // makes the inverted follower motors for the right side
    public CANSparkMax RightMain = createFollowerSparkMAXInverted(kElevatorRightMain);
    public CANSparkMax RightSecondary = createFollowerSparkMAXInverted(kElevatorRightSecondary);

    // Check if limit switch is forward or Reverse irl
    // Check if we are going to use Normally closed or Normally open
    // used to sync with encoders
    public SparkMaxLimitSwitch limitSwitchLeftMain = LeftMain.getForwardLimitSwitch(Type.kNormallyClosed);

    // Creates an incremental encoder
    public RelativeEncoder encoder = LeftMain.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
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

    // creates and sets a PID controller
    public SparkMaxPIDController controllerLeftMain = createAndSetupPIDController(LeftMain);

    // gets the number of rotations from the encoder
    public double getPosition() {
        return encoder.getPosition();
    }

    // Sets the setPoint of the PID controller to whatever was passed as the input
    public void setPosition(double length) {
        controllerLeftMain.setReference(length, CANSparkMax.ControlType.kPosition);
    }

    // resets the encoder
    public void zeroEncoder() {
        if (limitSwitchLeftMain.isPressed()) {
            encoder.setPosition(0);
        } else {
            System.out.println("limit switch is no pressy");
        }
    }
}
