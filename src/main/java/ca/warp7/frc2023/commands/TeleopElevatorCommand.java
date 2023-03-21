package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class TeleopElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private DoubleSupplier stickSup;

    public TeleopElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier stickSup) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.stickSup = stickSup;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        // Get values and apply deadband
        double stickValue = MathUtil.applyDeadband(stickSup.getAsDouble(), kTeleop.kElevatorStickDeadband);

        elevatorSubsystem.setSpeed(stickValue);
    }
}
