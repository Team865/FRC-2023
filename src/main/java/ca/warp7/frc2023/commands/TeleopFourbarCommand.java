package ca.warp7.frc2023.commands;

import ca.warp7.frc2023.Constants.kTeleop;
import ca.warp7.frc2023.subsystems.FourbarSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;

public class TeleopFourbarCommand extends CommandBase {
    private FourbarSubsystem fourbarSubsystem;

    private DoubleSupplier fourBarModifierSup;
    private double setPointModifier;

    public TeleopFourbarCommand(FourbarSubsystem fourbarSubsystem, DoubleSupplier fourBarModSup) {

        this.fourbarSubsystem = fourbarSubsystem;
        addRequirements(fourbarSubsystem);

        this.fourBarModifierSup = fourBarModSup;
    }

    @Override
    public void execute() {
        double fourbarModDeadbandApplied =
                MathUtil.applyDeadband(fourBarModifierSup.getAsDouble(), kTeleop.kStickDeadband);

        if (fourbarModDeadbandApplied > 0) {
            setPointModifier = 5;
        } else if (fourbarModDeadbandApplied < 0) {
            setPointModifier = -5;
        } else {
            setPointModifier = 0;
        }

        fourbarSubsystem.setSetPointModifier(setPointModifier);
    }
}
