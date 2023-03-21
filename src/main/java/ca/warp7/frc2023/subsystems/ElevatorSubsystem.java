package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftPrimaryMotor;
    private CANSparkMax leftSecondaryMotor;
    private CANSparkMax rightPrimaryMotor;
    private CANSparkMax rightSecondaryMotor;

    private SparkMaxLimitSwitch motorLimitSwitch;
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

        motorLimitSwitch = rightPrimaryMotor.getReverseLimitSwitch(Type.kNormallyClosed);

        motorController = new PIDController(0.5, 0, 0);

        encoder = new Encoder(0, 1, true);
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

    public void setSpeed(double speed) {
        if ((motorLimitSwitch.isPressed() && speed <= 0) || encoder.getDistance() < -0.5) {
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

    public Command startPosition() {
        return this.runOnce(() -> this.setPosition(0));
    }

    public Command mediumGoal() {
        return this.runOnce(() -> this.setPosition(5));
    }

    public Command highGoal() {
        return this.runOnce(() -> this.setPosition(10));
    }

    @Override
    public void periodic() {
        // setSpeed(motorController.calculate(encoder.getDistance(), setLength));

        SmartDashboard.putNumber("elevator encoder", encoder.getDistance());
        SmartDashboard.putBoolean("elevator limitswitch", motorLimitSwitch.isPressed());

        SmartDashboard.putNumber("Elevator set-to length", setLength);
        SmartDashboard.putNumber("Elevator current length", getPosition());
    }
}
