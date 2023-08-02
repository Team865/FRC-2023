package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class FourbarSubsystem extends TrapezoidProfileSubsystem {
    private final CANSparkMax motorLeft;
    private final CANSparkMax motorRight;
    private final RelativeEncoder builtInEncoder;
    private final SparkMaxPIDController motorController;

    public FourbarSubsystem() {
        super(new TrapezoidProfile.Constraints(120, 200));
        // Create right motor
        motorRight = new CANSparkMax(kFourbar.kMotorRightID, MotorType.kBrushless);

        // Create left motor
        motorLeft = new CANSparkMax(kFourbar.kMotorLeftID, MotorType.kBrushless);

        motorLeft.setCANTimeout(0);
        motorRight.setCANTimeout(0);

        motorRight.restoreFactoryDefaults();
        motorLeft.restoreFactoryDefaults();
        motorRight.setIdleMode(IdleMode.kBrake);
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.setInverted(true);

        motorRight.setSmartCurrentLimit(40);
        motorLeft.setSmartCurrentLimit(40);
        motorLeft.enableVoltageCompensation(12);
        motorRight.enableVoltageCompensation(12);

        motorRight.follow(motorLeft, true);

        builtInEncoder = motorLeft.getEncoder();
        motorController = motorLeft.getPIDController();

        setPID();
    }

    public void setPID() {
        motorController.setP(0.15);
        motorController.setI(0);
        motorController.setD(0);
        motorController.setFF(0);
        motorController.setIZone(0);
        motorController.setOutputRange(-1, 1);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Fourbar Current Position", builtInEncoder.getPosition());
        SmartDashboard.putNumber("Fourbar Applied Output", motorLeft.getAppliedOutput());
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        SmartDashboard.putNumber("Fourbar Set Point Position", setpoint.position);
        motorController.setReference(setpoint.position, ControlType.kPosition);
    }

    public Command setFourbarGoalCommand(double fourbarGoal) {
        return Commands.runOnce(() -> setGoal(fourbarGoal), this);
    }

    public Command setFourbarGoalCommand(TrapezoidProfile.State fourbarState) {
        return Commands.runOnce(() -> setGoal(fourbarState), this);
    }
}
