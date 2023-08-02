package ca.warp7.frc2023.lib.config;

import edu.wpi.first.math.controller.PIDController;

public class PIDConfig {
    public double p, i, d;

    public PIDConfig(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
    }

    public PIDController getController() {
        return new PIDController(p, i, d);
    }
}
