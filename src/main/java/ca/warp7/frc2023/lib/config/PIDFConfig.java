package ca.warp7.frc2023.lib.config;

public class PIDFConfig {
    public double p, i, d, f;

    public PIDFConfig(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }
}
