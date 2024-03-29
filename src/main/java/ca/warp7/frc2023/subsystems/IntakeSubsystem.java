package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.kIntake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax frontWheelMotor;
    private final CANSparkMax rearWheelMotor;
    private final CANSparkMax talonPivotMotor;

    private final RelativeEncoder talonPivotEncoder;
    private final SparkMaxPIDController talonPivotMotorController;

    private double talonPivotSetPoint;

    public IntakeSubsystem() {
        frontWheelMotor = new CANSparkMax(kIntake.kFrontWheelMotorID, MotorType.kBrushless);
        rearWheelMotor = new CANSparkMax(kIntake.kRearWheelMotorID, MotorType.kBrushless);
        talonPivotMotor = new CANSparkMax(kIntake.kTalonPivotMotorID, MotorType.kBrushless);

        frontWheelMotor.setCANTimeout(0);
        rearWheelMotor.setCANTimeout(0);
        talonPivotMotor.setCANTimeout(0);

        frontWheelMotor.restoreFactoryDefaults();
        rearWheelMotor.restoreFactoryDefaults();
        talonPivotMotor.restoreFactoryDefaults();

        frontWheelMotor.setIdleMode(IdleMode.kBrake);
        rearWheelMotor.setIdleMode(IdleMode.kBrake);
        talonPivotMotor.setIdleMode(IdleMode.kBrake);

        rearWheelMotor.setInverted(false);
        frontWheelMotor.setInverted(true);

        frontWheelMotor.setSmartCurrentLimit(20);
        rearWheelMotor.setSmartCurrentLimit(20);
        talonPivotMotor.setSmartCurrentLimit(15);

        talonPivotEncoder = talonPivotMotor.getEncoder();
        talonPivotMotorController = talonPivotMotor.getPIDController();

        zeroEncoder();
        configController();
    }

    private void configController() {
        talonPivotMotorController.setP(0.5);
        talonPivotMotorController.setI(0);
        talonPivotMotorController.setD(0);
        talonPivotMotorController.setIZone(0);
        talonPivotMotorController.setFF(0);
        talonPivotMotorController.setOutputRange(-0.5, 0.5);
    }

    public void setIntakeSpeed(double bothWheelSpeeds) {
        frontWheelMotor.set(bothWheelSpeeds);
        rearWheelMotor.set(bothWheelSpeeds);
    }

    public void setIntakeSpeed(double frontWheelSpeeds, double rearWheelSpeeds) {
        frontWheelMotor.set(frontWheelSpeeds);
        rearWheelMotor.set(rearWheelSpeeds);
    }

    public void zeroEncoder() {
        talonPivotEncoder.setPosition(0);
    }

    public double getPosition() {
        return talonPivotEncoder.getPosition();
    }

    public Command zeroEncoderCommand() {
        return runOnce(this::zeroEncoder);
    }

    public Command setIntakeSpeedCommand(double bothWheelSpeeds) {
        return this.runOnce(() -> this.setIntakeSpeed(bothWheelSpeeds));
    }

    public Command setTalonPivotSetPoint(double setPoint) {
        return this.runOnce(() -> this.talonPivotSetPoint = setPoint);
    }

    @Override
    public void periodic() {
        talonPivotMotorController.setReference(talonPivotSetPoint, ControlType.kPosition);
    }
}
