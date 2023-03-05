package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.subsystems.IntakeSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopIntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    private DoubleSupplier frontWheelSpeedSup, rearWheelSpeedSup, bothWheelSpeedSup, talonPivotSetPointSup;
    private BooleanSupplier controlSpeedsSeparateSup, reverseSpeedsSup;
    private double frontWheelSpeed, rearWheelSpeed, bothWheelSpeed;

    public TeleopIntakeCommand(
            IntakeSubsystem intakeSubsystem,
            BooleanSupplier controlSpeedsSeparateSup,
            BooleanSupplier reverseSpeedsSup,
            DoubleSupplier frontWheelSpeedSup,
            DoubleSupplier rearWheelSpeedSup,
            DoubleSupplier bothWheelSpeedSup,
            DoubleSupplier talonPivotSetPointSup) {

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);

        this.reverseSpeedsSup = reverseSpeedsSup;
        this.controlSpeedsSeparateSup = controlSpeedsSeparateSup;
        this.frontWheelSpeedSup = frontWheelSpeedSup;
        this.rearWheelSpeedSup = rearWheelSpeedSup;
        this.bothWheelSpeedSup = bothWheelSpeedSup;
        this.talonPivotSetPointSup = talonPivotSetPointSup;
    }

    @Override
    public void execute() {
        if (reverseSpeedsSup.getAsBoolean()) {
            frontWheelSpeed = MathUtil.applyDeadband(-frontWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);
            rearWheelSpeed = MathUtil.applyDeadband(-rearWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);
            bothWheelSpeed = MathUtil.applyDeadband(-bothWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);

        } else {
            frontWheelSpeed = MathUtil.applyDeadband(frontWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);
            rearWheelSpeed = MathUtil.applyDeadband(rearWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);
            bothWheelSpeed = MathUtil.applyDeadband(bothWheelSpeedSup.getAsDouble(), kTeleop.kTriggerDeadband);
        }

        if (controlSpeedsSeparateSup.getAsBoolean()) {
            intakeSubsystem.setIntakeSpeed(frontWheelSpeed, rearWheelSpeed);
        } else {
            intakeSubsystem.setIntakeSpeed(bothWheelSpeed);
        }
    }
}
