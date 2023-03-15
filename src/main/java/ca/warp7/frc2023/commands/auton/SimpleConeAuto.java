package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants.kAuton;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SimpleConeAuto extends SequentialCommandGroup {
    public SimpleConeAuto(IntakeSubsystem intakeSubsystem) {
        addCommands(
                intakeSubsystem.setIntakeSpeedCommand(kAuton.kOuttakeSpeed),
                Commands.waitSeconds(1.0),
                intakeSubsystem.setIntakeSpeedCommand(0.0));
    }
}
