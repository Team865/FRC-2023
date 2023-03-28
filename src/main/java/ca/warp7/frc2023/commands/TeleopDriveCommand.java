package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.lib.math.SensitivityGainAdjustment;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDriveCommand extends CommandBase {
    private SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;

    private DoubleSupplier translationSup, strafeSup, rotationSup;
    private BooleanSupplier isRobotOrientedSup, isSlowModeSup;

    public TeleopDriveCommand(
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier isRobotOrientedSup,
            BooleanSupplier isSlowModeSup) {

        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        addRequirements(swerveDrivetrainSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.isRobotOrientedSup = isRobotOrientedSup;
        this.isSlowModeSup = isSlowModeSup;
    }

    @Override
    public void execute() {

        // Get values and apply deadband
        double xMagnitude = SensitivityGainAdjustment.driveGainAdjustment(
                MathUtil.applyDeadband(translationSup.getAsDouble(), kTeleop.kDriveDeadband));
        double yMagnitude = SensitivityGainAdjustment.driveGainAdjustment(
                MathUtil.applyDeadband(strafeSup.getAsDouble(), kTeleop.kDriveDeadband));
        double rotationMagnitude = MathUtil.applyDeadband(rotationSup.getAsDouble(), kTeleop.kRotateDeadband);

        // Remove brake if high throttle
        if ((xMagnitude > 0.9 || yMagnitude > 0.9 || rotationMagnitude > 0.9)
                && swerveDrivetrainSubsystem.isBrakeEnabled) {
            swerveDrivetrainSubsystem.isBrakeEnabled = false;
        }

        if (isSlowModeSup.getAsBoolean()) {
            xMagnitude *= 0.5;
            yMagnitude *= 0.5;
            rotationMagnitude *= 0.5;
        }

        if (!swerveDrivetrainSubsystem.isBrakeEnabled) {
            // Drive
            swerveDrivetrainSubsystem.drive(
                    new Translation2d(xMagnitude, yMagnitude).times(kDrivetrain.kMaxSpeed),
                    rotationMagnitude * kDrivetrain.kMaxAngularVelocity,
                    !isRobotOrientedSup.getAsBoolean(),
                    true);
        }
    }
}
