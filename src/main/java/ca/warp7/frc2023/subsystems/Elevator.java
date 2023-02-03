package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
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

    // method that creates a master motor
    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    // method that creates a follower motor
    public static CANSparkMax createFollowerSparkMAX(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.follow(LeftMain);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // creates an inverted SparkMAx follower motor
    public static CANSparkMax createFollowerSparkMAXInverted(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        follower.restoreFactoryDefaults();
        follower.follow(LeftMain, true);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // makes a leader and a follower motor for the left side

    public static CANSparkMax LeftMain = createMasterSparkMAX(kElevatorLeftMain);
    public static CANSparkMax LeftSecondary = createFollowerSparkMAX(kElevatorLeftSecondary);

    // makes the inverted follower motors for the right side
    public static CANSparkMax RightMain = createFollowerSparkMAXInverted(kElevatorRightMain);
    public static CANSparkMax RightSecondary = createFollowerSparkMAXInverted(kElevatorRightSecondary);

    // sets the leader motor speed for either side
    public static void setSpeed(double speed) {
        LeftMain.set(speed);
        RightMain.set(speed);
    }

    // Check if limit switch is foward or Reverse irl
    // Check if we are going to use Normally closed or Normally open
    // used to sync with encoders
    public static SparkMaxLimitSwitch limitSwitchLeftMain = LeftMain.getForwardLimitSwitch(Type.kNormallyClosed);

    // Creates an abolute encoder for each motor
    // lol change encoder to incrmental because we are the goofy 
    public static SparkMaxAbsoluteEncoder absoluteEncoderLeftMain =
            LeftMain.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    // Method that creates and sets up a pid controller
    public static SparkMaxPIDController createAndSetupPIDController(CANSparkMax motor) {
        SparkMaxPIDController PIDController = motor.getPIDController();
        PIDController.setFeedbackDevice(absoluteEncoderLeftMain);
        double kP = 0.1;
        double kI = 1e-4;
        double kD = 1;
        double kIz = 0;
        double kFF = 0;
        double kMaxOutput = 1;
        double kMinOutput = -1;
        PIDController.setP(kP);
        PIDController.setI(kI);
        PIDController.setD(kD);
        PIDController.setIZone(kIz);
        PIDController.setFF(kFF);
        PIDController.setOutputRange(kMinOutput, kMaxOutput);
        PIDController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        return PIDController;
    }

    // creates and sets a PID controller
    public SparkMaxPIDController controllerLeftMain = createAndSetupPIDController(LeftMain);

    // gets the position of the absolute encoder
    public double getPosition() {
        return absoluteEncoderLeftMain.getPosition();
    }

    // allows you to extend the elevator
    public void setPosition(double length) {
        controllerLeftMain.setReference(length, CANSparkMax.ControlType.kPosition);
    }

    // resets the endoder
    public void zeroEncoder() {
        if (limitSwitchLeftMain.isPressed()) {
            absoluteEncoderLeftMain.setZeroOffset(getPosition());
        } else {
            System.out.println("limit switch is no pressy");
        }
    }

    // fully retracts elevatorqq
    public void fullyRetract() {
        setPosition(0);
        zeroEncoder();
    }
}