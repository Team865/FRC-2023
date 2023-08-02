package ca.warp7.frc2023.lib.math;

public class SensitivityGainAdjustment {

    public static double driveGainAdjustment(double input) {
        return ((0.45) * (input)) + ((.55) * (Math.pow(input, 3)));
    }

    public static double rotateGainAdjustment(double input) {
        return ((0.75) * (input)) + ((.25) * (Math.pow(input, 3)));
    }
}
