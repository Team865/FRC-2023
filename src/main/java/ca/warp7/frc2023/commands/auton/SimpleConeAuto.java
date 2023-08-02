package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleConeAuto extends SequentialCommandGroup {
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public SimpleConeAuto() {
        addCommands(
                intakeSubsystem.setIntakeSpeedCommand(0.5),
                Commands.waitSeconds(1.0),
                intakeSubsystem.setIntakeSpeedCommand(0.0));
    }
}
