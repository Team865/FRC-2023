package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class ElevatorSubsystem extends ProfiledPIDSubsystem {
    private final PWMSparkMax motorGroup;

    private final DigitalInput limitSwitch;
    // private final ArmFeedforward elevatorFeedforward;
    private final Encoder encoder;

    public Goals currentGoal;

    public ElevatorSubsystem() {
        super(kElevator.kProfiledPIDController.getController());

        motorGroup = new PWMSparkMax(9);

        encoder = new Encoder(kElevator.kEncoderIDs[0], kElevator.kEncoderIDs[1], kElevator.kEncoderReversed);
        encoder.setDistancePerPulse(kElevator.kEncoderDistancePerPulse);

        limitSwitch = new DigitalInput(kElevator.kLimitSwitchID);

        // Uses an ArmFeedForward because it works better. No clue why.
        // elevatorFeedforward = new ArmFeedforward(0.17334, 0.12817, 0.30618, 0.057124);

        currentGoal = Goals.NONE;
        setGoal(0);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public double getMeasurement() {
        return encoder.getDistance();
    }

    public Command setElevatorGoalCommand(double elevatorSetLength) {
        return runOnce(() -> {
            setGoal(elevatorSetLength);
            enable();
        });
    }

    public Command setElevatorGoalCommand(TrapezoidProfile.State elevatorSetState) {
        return runOnce(() -> {
            setGoal(elevatorSetState);
            enable();
        });
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("Current Real Goal", currentGoal.name());
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {

        double finalOutput = MathUtil.clamp(output, -12, 12);

        SmartDashboard.putNumber("Final Output", finalOutput);
        SmartDashboard.putNumber("Current measurement", getMeasurement());
        SmartDashboard.putNumber("Goal Position", getController().getGoal().position);
        SmartDashboard.putNumber("Goal Velocity", getController().getGoal().velocity);
        SmartDashboard.putNumber("Setpoint Position", getController().getSetpoint().position);
        SmartDashboard.putNumber("Setpoint Velocity", getController().getSetpoint().velocity);
        SmartDashboard.putBoolean("Elevator Enabled", isEnabled());
        SmartDashboard.putBoolean("Limit Switch Hit", getLimitSwitch());

        motorGroup.setVoltage(finalOutput);
    }

    public Command setCurrentGoal(Goals currentGoal) {
        return runOnce(() -> this.currentGoal = currentGoal);
    }
}
