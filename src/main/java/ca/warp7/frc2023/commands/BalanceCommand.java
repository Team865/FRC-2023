// Thanks 3161!
package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private PIDController balanceController = new PIDController(0.043, 0.0, 0, 2);

    public BalanceCommand(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem) {
        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        this.addRequirements(swerveDrivetrainSubsystem);
    }

    @Override
    public void execute() {
        double power = 0;
        SmartDashboard.putNumber("Pitch", swerveDrivetrainSubsystem.getPitch());
        power = this.balanceController.calculate(this.swerveDrivetrainSubsystem.getPitch(), 0);
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
