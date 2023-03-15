package ca.warp7.frc2023.commands.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface AutoImpl {
    Pose2d getInitialHolonomicPose();

    Command getCommand();
}
