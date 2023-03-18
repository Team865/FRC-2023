package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private CANSparkMax leftPrimaryMotor;
    private CANSparkMax leftSecondaryMotor;
    private CANSparkMax rightPrimaryMotor;
    private CANSparkMax rightSecondaryMotor;

    private SparkMaxLimitSwitch motorLimitSwitch;
    private SparkMaxPIDController motorController;
    private RelativeEncoder motorEncoder;
    double setLength = 0;

    public ElevatorSubsystem() {
        setLength = 0;

        leftPrimaryMotor = new CANSparkMax(kElevator.kLeftPrimaryMotorID, MotorType.kBrushed);
        leftSecondaryMotor = new CANSparkMax(kElevator.kLeftSecondaryMotorID, MotorType.kBrushed);
        rightPrimaryMotor = new CANSparkMax(kElevator.kRightPrimaryMotorID, MotorType.kBrushed);
        rightSecondaryMotor = new CANSparkMax(kElevator.kRightSecondaryMotorID, MotorType.kBrushed);

        leftPrimaryMotor.restoreFactoryDefaults();
        leftSecondaryMotor.restoreFactoryDefaults();
        rightPrimaryMotor.restoreFactoryDefaults();
        rightSecondaryMotor.restoreFactoryDefaults();

        leftPrimaryMotor.setIdleMode(IdleMode.kBrake);
        leftSecondaryMotor.setIdleMode(IdleMode.kBrake);
        rightPrimaryMotor.setIdleMode(IdleMode.kBrake);
        rightSecondaryMotor.setIdleMode(IdleMode.kBrake);

        leftPrimaryMotor.setSmartCurrentLimit(15);
        leftSecondaryMotor.setSmartCurrentLimit(15);
        rightPrimaryMotor.setSmartCurrentLimit(15);
        rightSecondaryMotor.setSmartCurrentLimit(15);

        leftPrimaryMotor.setInverted(true);

        leftSecondaryMotor.follow(leftPrimaryMotor, false);
        rightPrimaryMotor.follow(leftPrimaryMotor, true);
        rightSecondaryMotor.follow(leftPrimaryMotor, false);

        // motorLimitSwitch = rightPrimaryMotor.getReverseLimitSwitch(
        //         Type.kNormallyClosed); // TODO: change to correct motor and nomal type

        motorEncoder = leftPrimaryMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);

        motorController = leftPrimaryMotor.getPIDController();
        configController();
    }

    private void configController() {
        motorController.setP(0.1);
        motorController.setI(0);
        motorController.setD(0);
        motorController.setIZone(0);
        motorController.setFF(0);
        motorController.setOutputRange(-0.6, 0.6);
    }

    public void setSpeed(double speed) {
        leftPrimaryMotor.set(speed * 0.8);
    }

    private double getPosition() {
        // return 4 * (motorEncoder.getPosition() * 1.751);
        return motorEncoder.getPosition();
    }

    private void setPosition(double length) {
        // double numOfRotationsHexShaft = ((1.751 * 4) - length) / 4;
        // double numOfRotations = numOfRotationsHexShaft * 97.5;
        setLength = length;
    }

    private void zeroEncoder() {
        motorEncoder.setPosition(0);
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
        // motorController.setReference(setLength, ControlType.kPosition);
        // if (motorLimitSwitch.isPressed()) {
        //     zeroEncoder();
        // }

        SmartDashboard.putNumber("Elevator set-to length", setLength);
        SmartDashboard.putNumber("Elevator current length", getPosition());
    }
}
