package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants.kAuton;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MobilityCone extends SequentialCommandGroup {
    public MobilityCone(IntakeSubsystem intakeSubsystem, SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {
        addCommands(
                intakeSubsystem.setIntakeSpeedCommand(kAuton.kOuttakeSpeed),
                Commands.waitSeconds(1.0),
                intakeSubsystem.setIntakeSpeedCommand(0.0),
                swerveDrivetrainSubsystem.mobilty().withTimeout(5.5));
    }
}
