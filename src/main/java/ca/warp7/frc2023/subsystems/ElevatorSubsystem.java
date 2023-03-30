package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftPrimaryMotor;
    private final CANSparkMax leftSecondaryMotor;
    private final CANSparkMax rightPrimaryMotor;
    private final CANSparkMax rightSecondaryMotor;

    private DigitalInput limitSwitch;
    private PIDController profiledPIDController;
    private Encoder encoder;

    private boolean safetyTriggered;
    private double setPosition;

    public ElevatorSubsystem() {
        leftPrimaryMotor = new CANSparkMax(kElevator.kLeftPrimaryMotorID, MotorType.kBrushed);
        leftSecondaryMotor = new CANSparkMax(kElevator.kLeftSecondaryMotorID, MotorType.kBrushed);
        rightPrimaryMotor = new CANSparkMax(kElevator.kRightPrimaryMotorID, MotorType.kBrushed);
        rightSecondaryMotor = new CANSparkMax(kElevator.kRightSecondaryMotorID, MotorType.kBrushed);

        //        comment out and check
        //        leftSecondaryMotor.follow(leftPrimaryMotor);
        //        rightPrimaryMotor.follow(leftPrimaryMotor);
        //        rightSecondaryMotor.follow(leftPrimaryMotor);

        leftPrimaryMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        leftSecondaryMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        rightPrimaryMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);
        rightSecondaryMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 10);

        configMotor(leftPrimaryMotor);
        configMotor(leftSecondaryMotor);
        configMotor(rightPrimaryMotor);
        configMotor(rightSecondaryMotor);

        limitSwitch = new DigitalInput(2);

        profiledPIDController = new PIDController(0, 0, 0);
        configProfiledPIDController();

        encoder = new Encoder(0, 1, false);
        encoder.setDistancePerPulse(1 / (24.38 * 2 * Math.PI * 0.8755));
        encoder.reset();
    }

    private void configMotor(CANSparkMax sparkMax) {
        sparkMax.setIdleMode(IdleMode.kBrake);
        sparkMax.setSmartCurrentLimit(20);
        sparkMax.enableVoltageCompensation(kBatteryNominalVoltage);
        sparkMax.setCANTimeout(0);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, 500);
        sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, 500);
        sparkMax.burnFlash();
    }

    private void configProfiledPIDController() {
        profiledPIDController.setP(0.25);
        profiledPIDController.setI(0);
        profiledPIDController.setD(0);
        profiledPIDController.setTolerance(1);
    }

    public boolean isAtPosition(double setPointRadius) {
        profiledPIDController.setTolerance(setPointRadius);
        return profiledPIDController.atSetpoint();
    }

    /**
     * @return the position the elevator is at (in inches)
     */
    private double getPosition() {
        return encoder.getDistance();
    }

    /**
     * Zeroes the encoder on the elevator
     */
    private void zeroEncoder() {
        encoder.reset();
    }

    /**
     * @param position the position to set the elevator to (in inches)
     * @return runOnce command
     */
    public Command setPositionCommand(double position) {
        return runOnce(() -> setPosition = position);
    }

    /**
     * Will not run the motor if: hitting limit switch, thinks it's below the lowest, or thinks it's above the highest
     * @param speed percent to run the elevator motors at
     */
    public void setSpeed(double speed) {
        if ((limitSwitch.get() && speed <= 0) || getPosition() < -0.5) {
            leftPrimaryMotor.set(0);
            leftSecondaryMotor.set(0);
            rightPrimaryMotor.set(0);
            rightSecondaryMotor.set(0);
            safetyTriggered = true;
            zeroEncoder();
        } else {
            safetyTriggered = false;
            leftPrimaryMotor.set(speed);
            leftSecondaryMotor.set(speed);
            rightPrimaryMotor.set(speed);
            rightSecondaryMotor.set(speed);
        }
    }

    @Override
    public void periodic() {
        double outputPercent = MathUtil.clamp(profiledPIDController.calculate(getPosition(), setPosition), -1, 1);
        setSpeed(outputPercent);
        SmartDashboard.putNumber("leftPrimaryMotor applied", leftPrimaryMotor.getAppliedOutput());
        SmartDashboard.putNumber("leftSecondaryMotor applied", leftSecondaryMotor.getAppliedOutput());
        SmartDashboard.putNumber("rightPrimaryMotor applied", rightPrimaryMotor.getAppliedOutput());
        SmartDashboard.putNumber("rightSecondaryMotor applied", rightSecondaryMotor.getAppliedOutput());

        SmartDashboard.putNumber("Elevator Percent Output (%)", outputPercent);
        SmartDashboard.putNumber("Elevator Position (m)", getPosition());
        SmartDashboard.putBoolean("Elevator Limit Switch Activated", limitSwitch.get());
        SmartDashboard.putBoolean("Elevator Safety Triggered", safetyTriggered);
        SmartDashboard.putNumber("Elevator Set-to Length", setPosition);
    }
}
