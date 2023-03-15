package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourbarSubsystem extends SubsystemBase {
    private CANSparkMax motorLeft;
    private CANSparkMax motorRight;
    private RelativeEncoder builtInEncoder;
    private SparkMaxPIDController motorController;
    private double position;
    private double setPointModifier;

    public FourbarSubsystem() {
        position = 0;
        setPointModifier = 0;

        // Create right motor
        motorRight = new CANSparkMax(kFourbar.kFourbarMotorRightID, MotorType.kBrushless);

        // Create left motor
        motorLeft = new CANSparkMax(kFourbar.kFourbarMotorLeftID, MotorType.kBrushless);

        motorRight.restoreFactoryDefaults();
        motorLeft.restoreFactoryDefaults();
        motorRight.setIdleMode(IdleMode.kBrake);
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.setInverted(true);

        motorRight.setSmartCurrentLimit(40);
        motorLeft.setSmartCurrentLimit(40);

        motorRight.follow(motorLeft, true);

        builtInEncoder = motorLeft.getEncoder();
        motorController = motorLeft.getPIDController();

        setPID();
    }

    // Get position of motor
    public double getPosition() {
        return builtInEncoder.getPosition();
    }

    // Zeroes the encoder
    public void zeroEncoder() {
        builtInEncoder.setPosition(0);
    }

    // Set the position of the motor
    public Command setPosition(double setPosition) {
        return this.runOnce(() -> position = setPosition);
    }

    public void setPID() {
        motorController.setP(0.3);
        motorController.setI(0);
        motorController.setD(0);
        motorController.setFF(0);
        motorController.setIZone(0);
        motorController.setOutputRange(-0.4, 0.5);
    }

    public void setSetPointModifier(double modifier) {
        this.setPointModifier = modifier;
    }

    public void configBuiltInEncoder() {
        // builtInEncoder.setPositionConversionFactor(kFourbar.kConversionFactor);
        zeroEncoder();
    }

    public void configLeftMotor() {
        motorLeft.restoreFactoryDefaults();
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.enableVoltageCompensation(12);
        motorLeft.setSmartCurrentLimit(60);
        motorLeft.follow(motorRight, true);
    }

    public void configRightMotor() {
        motorRight.restoreFactoryDefaults();
        motorRight.setIdleMode(IdleMode.kBrake);
        motorRight.enableVoltageCompensation(12);
        motorRight.setSmartCurrentLimit(60);
    }

    @Override
    public void periodic() {
        // System.out.println(position);
        motorController.setReference(position + setPointModifier, ControlType.kPosition);
        SmartDashboard.putNumber("Fourbar set-to position", position);
        SmartDashboard.putNumber("Fourbar current position", getPosition());
    }
}
