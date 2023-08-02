package ca.warp7.frc2023.commands.auton;

import ca.warp7.frc2023.Constants.*;
import ca.warp7.frc2023.commands.BalanceCommand;
import ca.warp7.frc2023.commands.SetGoalCommands;
import ca.warp7.frc2023.subsystems.ElevatorSubsystem;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.HashMap;

public class ConfiguredSwerveAutoBuilder {
    HashMap<String, Command> eventMap = new HashMap<>();
    SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    FourbarSubsystem fourbarSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;

    public ConfiguredSwerveAutoBuilder(
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem,
            FourbarSubsystem fourbarSubsystem,
            ElevatorSubsystem elevatorSubsystem,
            IntakeSubsystem intakeSubsystem) {

        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        this.fourbarSubsystem = fourbarSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        eventMap.put(
                "ScoreHighCone",
                Commands.sequence(
                        intakeSubsystem.setIntakeSpeedCommand(-0.3),
                        SetGoalCommands.highGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem),
                        new WaitCommand(2),
                        intakeSubsystem.setIntakeSpeedCommand(0.5),
                        new WaitCommand(0.2),
                        intakeSubsystem.setIntakeSpeedCommand(0.0),
                        SetGoalCommands.coneStow(
                                fourbarSubsystem,
                                elevatorSubsystem,
                                intakeSubsystem,
                                kFourbar.kConeCubeStowFromScoreWait,
                                0),
                        new WaitCommand(2)));
        eventMap.put(
                "ScoreMidCone",
                Commands.sequence(
                        intakeSubsystem.setIntakeSpeedCommand(kIntake.kFeedforwardSpeed),
                        SetGoalCommands.midGoal(fourbarSubsystem, elevatorSubsystem, intakeSubsystem),
                        new WaitCommand(2),
                        intakeSubsystem.setIntakeSpeedCommand(kIntake.kOuttakeSpeed),
                        new WaitCommand(0.2),
                        intakeSubsystem.setIntakeSpeedCommand(0.0),
                        SetGoalCommands.coneStow(
                                fourbarSubsystem,
                                elevatorSubsystem,
                                intakeSubsystem,
                                kFourbar.kConeCubeStowFromScoreWait,
                                0),
                        new WaitCommand(2)));
        eventMap.put(
                "GroundPickup",
                Commands.sequence(
                        SetGoalCommands.groundPickup(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0),
                        intakeSubsystem.setIntakeSpeedCommand(-0.8).withTimeout(0.5)));
        eventMap.put(
                "StowCone",
                Commands.sequence(
                        SetGoalCommands.coneStow(fourbarSubsystem, elevatorSubsystem, intakeSubsystem, 0, 0),
                        intakeSubsystem.setIntakeSpeedCommand(-0.3).withTimeout(1)));
        eventMap.put("AutoBalance", new BalanceCommand(swerveDrivetrainSubsystem, false).withTimeout(5));
    }

    public SwerveAutoBuilder getConfiguredAutoBuilder() {
        return new SwerveAutoBuilder(
                swerveDrivetrainSubsystem::getPose,
                swerveDrivetrainSubsystem::resetOdometry,
                kDrivetrain.kSwerveDriveKinematics,
                kAuton.kTranslationalController,
                kAuton.kRotationalController,
                swerveDrivetrainSubsystem::setModuleStates,
                eventMap,
                true,
                swerveDrivetrainSubsystem);
    }
}
