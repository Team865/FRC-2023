package ca.warp7.frc2023.commands;

import static ca.warp7.frc2023.Constants.kTeleop.kDriveSingleDirectionPercent;

import ca.warp7.frc2023.Constants.kDrivetrain;
import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.lib.math.SensitivityGainAdjustment;
import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class TeleopDriveCommand extends CommandBase {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;

    private final DoubleSupplier translationSup, strafeSup, rotationSup;
    private final BooleanSupplier isRobotOrientedSup, isSlowModeSup;
    private final IntSupplier POVSup;

    private double xMagnitude, yMagnitude, rotationMagnitude;

    public TeleopDriveCommand(
            SwerveDrivetrainSubsystem swerveDrivetrainSubsystem,
            DoubleSupplier translationSup,
            DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier isRobotOrientedSup,
            BooleanSupplier isSlowModeSup,
            IntSupplier POVSup) {

        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        addRequirements(swerveDrivetrainSubsystem);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.isRobotOrientedSup = isRobotOrientedSup;
        this.isSlowModeSup = isSlowModeSup;
        this.POVSup = POVSup;
    }

    @Override
    public void execute() {

        switch (POVSup.getAsInt()) {
            case (-1):
                xMagnitude = SensitivityGainAdjustment.driveGainAdjustment(
                        MathUtil.applyDeadband(translationSup.getAsDouble(), kTeleop.kDriveDeadband));
                yMagnitude = SensitivityGainAdjustment.driveGainAdjustment(
                        MathUtil.applyDeadband(strafeSup.getAsDouble(), kTeleop.kDriveDeadband));
                rotationMagnitude = SensitivityGainAdjustment.rotateGainAdjustment(
                        MathUtil.applyDeadband(rotationSup.getAsDouble(), kTeleop.kRotateDeadband));
                break;
            case (0):
                xMagnitude = kDriveSingleDirectionPercent;
                yMagnitude = 0;
                rotationMagnitude = 0;
                break;
            case (90):
                yMagnitude = -kDriveSingleDirectionPercent;
                xMagnitude = 0;
                rotationMagnitude = 0;
                break;

            case (180):
                xMagnitude = -kDriveSingleDirectionPercent;
                yMagnitude = 0;
                rotationMagnitude = 0;
                break;

            case (270):
                yMagnitude = kDriveSingleDirectionPercent;
                xMagnitude = 0;
                rotationMagnitude = 0;
                break;
        }

        if (isSlowModeSup.getAsBoolean()) {
            xMagnitude *= 0.5;
            yMagnitude *= 0.5;
            rotationMagnitude *= 0.5;
        }

        if (!swerveDrivetrainSubsystem.isBrakeEnabled()) {
            swerveDrivetrainSubsystem.drive(
                    new Translation2d(xMagnitude, yMagnitude).times(kDrivetrain.kMaxSpeed),
                    rotationMagnitude * kDrivetrain.kMaxAngularVelocity,
                    !isRobotOrientedSup.getAsBoolean(),
                    true);
        }
    }
}
