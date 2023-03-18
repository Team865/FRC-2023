package ca.warp7.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import ca.warp7.frc2023.Constants;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
public class SimpleConeAuto extends SequentialCommandGroup {
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    
    public SimpleConeAuto() {
        addCommands(
            intakeSubsystem.setIntakeSpeedCommand(Constants.kIntake.kAutoIntakeSpeed),
            Commands.waitSeconds(1.0),
            intakeSubsystem.setIntakeSpeedCommand(0.0)
        );  
    }    
}
