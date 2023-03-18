package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    private DoubleSupplier frontWheelSpeedSup, rearWheelSpeedSup, bothWheelSpeedSup;
    private BooleanSupplier isCubeIntakeSup, reverseSpeedsSup;
    private double frontWheelSpeed, rearWheelSpeed, bothWheelSpeed;

    public TeleopIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            BooleanSupplier isCubeIntakeSup,
            BooleanSupplier reverseSpeedsSup,
            DoubleSupplier bothWheelSpeedSup) {

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

        this.reverseSpeedsSup = reverseSpeedsSup;
        this.isCubeIntakeSup = isCubeIntakeSup;

        this.bothWheelSpeedSup = bothWheelSpeedSup;
    }

    @Override
    public void execute() {
        bothWheelSpeed = MathUtil.applyDeadband(bothWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);

        bothWheelSpeed = reverseSpeedsSup.getAsBoolean() ? bothWheelSpeed * -1 : bothWheelSpeed;

        if (isCubeIntakeSup.getAsBoolean()) {
            intakeSubsystem.setIntakeSpeed(frontWheelSpeed, -rearWheelSpeed);
        } else {
            intakeSubsystem.setIntakeSpeed(bothWheelSpeed);
        }
    }
}
