package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    // returns the instance of the subsystem
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    private static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    private CANSparkMax createFollowerSparkMAX(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(motorFirst);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // creates an inverted SparkMAx follower motor
    private CANSparkMax createFollowerSparkMAXInverted(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(motorFirst, true);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // makes a leader and a follower motor for the left side
    private CANSparkMax motorFirst = createMasterSparkMAX(kMotorFirst);
    private CANSparkMax motorSecond = createFollowerSparkMAX(kMotorSecond);

    // makes the inverted follower motors for the right side
    private CANSparkMax motorThird = createFollowerSparkMAXInverted(kMotorThird);
    private CANSparkMax motorFourth = createFollowerSparkMAXInverted(kMotorFourth);

    // Check if limit switch is forward or Reverse irl
    // Check if we are going to use Normally closed or Normally open
    // used to sync with encoders
    private SparkMaxLimitSwitch limitSwitchMotorFirst = motorFirst.getForwardLimitSwitch(Type.kNormallyClosed);

    // Creates an incremental encoder
    private RelativeEncoder encoder = motorFirst.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
    // Method that creates and sets up a PID controller
    private SparkMaxPIDController createAndSetupPIDController(CANSparkMax motor) {
        SparkMaxPIDController PIDController = motor.getPIDController();
        PIDController.setFeedbackDevice(encoder);
        double kP = 0.1;
        double kI = 0;
        double kD = 0;
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

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", kMaxVel);
        SmartDashboard.putNumber("Min Velocity", kMinVel);
        SmartDashboard.putNumber("Max Acceleration", kMaxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", kMinOutput);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", true);
        return PIDController;
    }

    // creates and sets a PID controller
    private SparkMaxPIDController controllerMotorFirst = createAndSetupPIDController(motorFirst);

    // gets the number of rotations from the encoder
    private double getPosition() {
        return 4*(encoder.getPosition()*1.175);
    }

    // Sets the setPoint of the PID controller to whatever was passed as the input
    private void setPosition(double length) {
        double numOfRotationsHexShaft = (7.0004 - length)/4;
        double numOfRotations = numOfRotationsHexShaft * 97.5;
        controllerMotorFirst.setReference(numOfRotations, CANSparkMax.ControlType.kPosition);
    }

    // resets the encoder
    private void zeroEncoder() {
        if (limitSwitchMotorFirst.isPressed()) {
            encoder.setPosition(0);
        } else {
            System.out.println("limit switch is no pressy");
        }
    }

    // fully retracts elevator
    private void fullyRetract() {
        setPosition(0);
        zeroEncoder();
    }

    @Override
    public void periodic() {
        fullyRetract();
    }
}
