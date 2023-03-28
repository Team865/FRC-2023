package ca.warp7.frc2023.lib.math;

public class SensitivityGainAdjustment {

    public static double driveGainAdjustment(double input) {
        return ((0.35) * (input)) + ((.65) * (Math.pow(input, 3)));
    }
}
