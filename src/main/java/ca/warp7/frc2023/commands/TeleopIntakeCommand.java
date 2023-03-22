package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    private DoubleSupplier coneIntakeSupplier, cubeIntakeSupplier;
    private BooleanSupplier outtakeSupplier;
    private double intakeSpeed;

    public TeleopIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            DoubleSupplier coneIntakeSupplier,
            DoubleSupplier cubeIntakeSupplier,
            BooleanSupplier outtakeSupplier) {

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

        this.coneIntakeSupplier = coneIntakeSupplier;
        this.cubeIntakeSupplier = cubeIntakeSupplier;
        this.outtakeSupplier = outtakeSupplier;
    }

    @Override
    public void execute() {
        if (cubeIntakeSupplier.getAsDouble() > coneIntakeSupplier.getAsDouble()) {
            intakeSpeed = cubeIntakeSupplier.getAsDouble();
            intakeSpeed *= outtakeSupplier.getAsBoolean() ? -1 : 1;
            intakeSubsystem.setIntakeSpeed(intakeSpeed);
        } else {
            intakeSpeed = coneIntakeSupplier.getAsDouble();
            intakeSpeed *= outtakeSupplier.getAsBoolean() ? -1 : 1;
            intakeSubsystem.setIntakeSpeed(intakeSpeed, -intakeSpeed);
        }
    }
}
