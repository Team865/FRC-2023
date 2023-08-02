// Thanks 3161!
package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.subsystems.SwerveDrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BalanceCommand extends CommandBase {
    private final SwerveDrivetrainSubsystem swerveDrivetrainSubsystem;
    private final PIDController balanceController = new PIDController(0.033, 0.0, 0.0, 0.02);
    private boolean isFieldOriented = true;

    public BalanceCommand(SwerveDrivetrainSubsystem swerveDrivetrainSubsystem, boolean isFieldOriented) {
        this.swerveDrivetrainSubsystem = swerveDrivetrainSubsystem;
        this.addRequirements(swerveDrivetrainSubsystem);
        this.isFieldOriented = isFieldOriented;
        balanceController.setTolerance(0.1);
    }

    @Override
    public void execute() {
        double power = this.balanceController.calculate(swerveDrivetrainSubsystem.getPitch(), 0);
        SmartDashboard.putNumber("Power", power);
        this.swerveDrivetrainSubsystem.drive(new Translation2d(power, 0), 0.0, isFieldOriented, true);
    }

    public boolean isBalanced() {
        double value = this.swerveDrivetrainSubsystem.getPitch();
        return Math.abs(value) <= balanceController.getPositionTolerance();
    }
}
