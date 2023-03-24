package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftPrimaryMotor;
    private CANSparkMax leftSecondaryMotor;
    private CANSparkMax rightPrimaryMotor;
    private CANSparkMax rightSecondaryMotor;

    private DigitalInput motorLimitSwitch;
    private PIDController motorController;
    private Encoder encoder;
    private MotorFeedbackSensor motorFeedbackSensor;
    double setLength;

    public ElevatorSubsystem() {
        setLength = 0;

        leftPrimaryMotor = new CANSparkMax(kElevator.kLeftPrimaryMotorID, MotorType.kBrushed);
        leftSecondaryMotor = new CANSparkMax(kElevator.kLeftSecondaryMotorID, MotorType.kBrushed);
        rightPrimaryMotor = new CANSparkMax(kElevator.kRightPrimaryMotorID, MotorType.kBrushed);
        rightSecondaryMotor = new CANSparkMax(kElevator.kRightSecondaryMotorID, MotorType.kBrushed);

        leftSecondaryMotor.follow(leftPrimaryMotor);
        rightPrimaryMotor.follow(leftPrimaryMotor);
        rightSecondaryMotor.follow(leftPrimaryMotor);

        // leftPrimaryMotor.restoreFactoryDefaults();
        // leftSecondaryMotor.restoreFactoryDefaults();
        // rightPrimaryMotor.restoreFactoryDefaults();
        // rightSecondaryMotor.restoreFactoryDefaults();

        leftPrimaryMotor.setIdleMode(IdleMode.kBrake);
        leftSecondaryMotor.setIdleMode(IdleMode.kBrake);
        rightPrimaryMotor.setIdleMode(IdleMode.kBrake);
        rightSecondaryMotor.setIdleMode(IdleMode.kBrake);

        // leftPrimaryMotor.setSmartCurrentLimit(15);
        // leftSecondaryMotor.setSmartCurrentLimit(15);
        // rightPrimaryMotor.setSmartCurrentLimit(15);
        // rightSecondaryMotor.setSmartCurrentLimit(15);

        // leftPrimaryMotor.setInverted(true);
        // rightPrimaryMotor.setInverted(false);

        // leftSecondaryMotor.follow(leftPrimaryMotor, true);
        // rightSecondaryMotor.follow(rightPrimaryMotor);

        motorLimitSwitch = new DigitalInput(2);

        motorController = new PIDController(0.25, 0, 0);
        motorController.setTolerance(1);

        encoder = new Encoder(0, 1, false);
        encoder.reset();
        encoder.setDistancePerPulse(1 / (24.38 * 2 * Math.PI * 0.8755));

        // configController();
    }

    private void configMotor() {}

    private void configController() {
        motorController.setP(0.1);
        motorController.setI(0);
        motorController.setD(0);
    }

    public boolean isAtPosition(double setPointRadius) {
        motorController.setTolerance(setPointRadius);
        return motorController.atSetpoint();
    }

    public void setSpeed(double speed) {
        if ((motorLimitSwitch.get() && speed <= 0) || encoder.getDistance() < -0.5) {
            leftPrimaryMotor.set(0);
            encoder.reset();
        } else {
            leftPrimaryMotor.set(speed);
        }

        // System.out.println(leftPrimaryMotor.getOutputCurrent());
        // System.out.println(leftPrimaryMotor.getVoltageCompensationNominalVoltage());
        // rightPrimaryMotor.set(speed);
    }

    private double getPosition() {
        // return 4 * (motorEncoder.getPosition() * 1.751);
        // return motorEncoder.getPosition();
        return 0;
    }

    private void setPosition(double length) {
        setLength = length;
    }

    private void zeroEncoder() {
        // motorEncoder.setPosition(0);
    }

    public Command setPositionCommand(double length) {
        return runOnce(() -> setLength = length);
    }

    @Override
    public void periodic() {
        double speed = motorController.calculate(encoder.getDistance(), setLength);

        setSpeed(speed);

        SmartDashboard.putNumber("elevator speed", speed);
        SmartDashboard.putNumber("elevator encoder", encoder.getDistance());
        SmartDashboard.putBoolean("elevator limitswitch", motorLimitSwitch.get());

        SmartDashboard.putNumber("Elevator set-to length", setLength);
    }
}
