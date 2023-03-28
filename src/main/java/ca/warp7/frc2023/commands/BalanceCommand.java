// Thanks 3161!
package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private PIDController balanceController = new PIDController(0.033, 0.0, 0.0);
    private boolean isFieldOriented = true;

    public BalanceCommand(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem, boolean isFieldOriented) {
        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        this.addRequirements(swerveDrivetrainSubsystem);
        this.isFieldOriented = isFieldOriented;
    }

    @Override
    public void execute() {
        double power = 0;
        SmartDashboard.putNumber("Pitch", Math.abs(swerveDrivetrainSubsystem.getPitch()));
        power = this.balanceController.calculate(swerveDrivetrainSubsystem.getPitch(), 0);
        SmartDashboard.putNumber("Power", power);
        this.swerveDrivetrainSubsystem.drive(new Translation2d(power, 0), 0.0, false, true);
    }

    public boolean isBalanced() {
        double value = this.swerveDrivetrainSubsystem.getPitch();
        return Math.abs(value) <= balanceController.getPositionTolerance();
    }

    @Override
    public void end(boolean interrupted) {}
}
