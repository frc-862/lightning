package com.lightningrobotics.common.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public final class LightningMath {

    /**
     * Private constructor
     */
    private LightningMath() {
        throw new AssertionError("utility class");
    }

    /**
     * Returns value clamped between low and high boundaries.
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value clamped between low and high boundaries.
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param highThe higher boundary to which to clamp value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns modulus of input.
     * @param input Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((input - minimumInput) / modulus);
        input -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((input - maximumInput) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     * @param angleRadians Angle to wrap in radians.
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
    }

    /**
     * Finds the maximum value of the given inputs.
     * @param values Multiple floating-point values.
     * @return The maximum of the values given.
     */
    public static double max(double... values) {
        var max = Double.MIN_VALUE;
        for (var v : values)
            if (v > max)
                max = v;
        return max;
    }

    public static int constrain(int value, int low, int high) {
      return Math.max(low, Math.min(value, high));
    }

    public static double constrain(double value, double low, double high) {
      return Math.max(low, Math.min(value, high));
    }
    public static double limit(double v, double low, double high) {
        return (v < low) ? low : ((v > high) ? high : v);
    }

    public static double limit(double v, double limit) {
        return limit(v, -limit, limit);
    }

    public static double limit(double input) {
        return limit(input, -1, 1);
    }

    public static double boundThetaNegPiToPi(double theta) {
        return theta - (Math.ceil((theta + Math.PI) / (Math.PI * 2)) - 1) * (Math.PI * 2); // (-π;π]
    }

    public static double boundTheta0To2Pi(double theta) {
        return theta - Math.floor(theta / (Math.PI * 2)) * (Math.PI * 2); // [0;2π)
    }

    public static double boundThetaNeg180to180(double theta) {
        return theta - (Math.ceil((theta + 180)/360)-1)*360; // (-180;180]
    }

    public static double boundTheta0to360(double theta) {
        return theta - Math.floor(theta/360)*360;  // [0;360)
    }

    public static double deltaThetaInDegrees(double from, double to) {
        return boundThetaNeg180to180(to - from);
    }

    public static double deltaThetaInRadians(double from, double to) {
        return boundThetaNegPiToPi(to - from);
    }

    public static int scale(int input,
                            int lowInput, int highInput, int lowOutput, int highOutput) {
        final int inputRange = highInput - lowInput;
        final int outputRange = highOutput - lowOutput;

        return (input - lowInput) * outputRange / inputRange + lowOutput;
    }

    public static double scale(double input,
                               double lowInput, double highInput, double lowOutput, double highOutput) {
        final double inputRange = highInput - lowInput;
        final double outputRange = highOutput - lowOutput;

        return  (input - lowInput) * outputRange / inputRange + lowOutput;
    }

    public static double deadZone(double input, double deadband) {
        return Math.abs(input) >= deadband ? input : 0;
    }

    /**
     * Linearly interpolates between two values.
     *
     * @param startValue The start value.
     * @param endValue   The end value.
     * @param t          The fraction for interpolation.
     * @return The interpolated value.
     */
    public static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    /**
     * Linearly interpolates between two poses.
     *
     * @param startValue The start pose.
     * @param endValue   The end pose.
     * @param t          The fraction for interpolation.
     * @return The interpolated pose.
     */
    public static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
        return startValue.plus((endValue.minus(startValue)).times(t));
    }

}
