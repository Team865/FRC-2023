package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
    double setLength;

    public ElevatorSubsystem() {
        leftPrimaryMotor = new CANSparkMax(kElevator.kLeftPrimaryMotorID, MotorType.kBrushed);
        leftSecondaryMotor = new CANSparkMax(kElevator.kLeftSecondaryMotorID, MotorType.kBrushed);
        rightPrimaryMotor = new CANSparkMax(kElevator.kRightPrimaryMotorID, MotorType.kBrushed);
        rightSecondaryMotor = new CANSparkMax(kElevator.kRightSecondaryMotorID, MotorType.kBrushed);

        leftSecondaryMotor.follow(leftPrimaryMotor);
        rightPrimaryMotor.follow(leftPrimaryMotor);
        rightSecondaryMotor.follow(leftPrimaryMotor);

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

    public void setSpeed(double speed) {
        if ((limitSwitch.get() && speed <= 0) || encoder.getDistance() < -0.5) {
            leftPrimaryMotor.set(0);
            encoder.reset();
        } else {
            leftPrimaryMotor.set(speed);
        }
    }

    private double getPosition() {
        return encoder.getDistance();
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
        double outputPercent = MathUtil.clamp(profiledPIDController.calculate(encoder.getDistance(), setLength), 0, 1);
        setSpeed(outputPercent);

        SmartDashboard.putNumber("Elevator Output Percent (%)", outputPercent);
        SmartDashboard.putNumber("Elevator Position (m)", getPosition());
        SmartDashboard.putBoolean("elevator limit switch", limitSwitch.get());

        SmartDashboard.putNumber("Elevator set-to length", setLength);
    }
}
