package ca.warp7.frc2023.lib.config;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ProfiledPIDConfig {
    public double p, i, d, maxV, maxA, tolerance;

    public ProfiledPIDConfig(double p, double i, double d, double maxV, double maxA, double tolerance) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxV = maxV;
        this.maxA = maxA;
        this.tolerance = tolerance;
    }

    public ProfiledPIDConfig(double p, double i, double d, double maxV, double maxA) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxV = maxV;
        this.maxA = maxA;
        this.tolerance = 0.5;
    }

    public ProfiledPIDController getController() {
        ProfiledPIDController controller =
                new ProfiledPIDController(p, i, d, new TrapezoidProfile.Constraints(maxV, maxA));
        controller.setTolerance(tolerance);
        return controller;
    }
}
