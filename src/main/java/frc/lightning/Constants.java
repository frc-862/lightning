package frc.lightning;

import frc.lightning.util.LightningMath;

public class Constants {
    
    // DEVELOPMENT
    public static final boolean TUNING_ENABLED = true;

    // HARDWARE
    public static final double TICS_PER_ROTATION = 2048; // 4 * 360;
    public static final double NEO_TICKS_PER_REV = 42;
    public static final int NEO_MAX_RPM = 5700;
    public static final double VOLT_LIMIT = 12d;
    public static final double WHEEL_DIAMETER_INCHES = 6;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double GEAR_REDUCTION = 15;
    public static final double WHEEL_DIAMETER = 6;
    public static final int firstPCSensor = 0;
    public static final double ENCODER_PULSES_PER_REVOLUTION = 2048;
    public static final double ENCODER_PULSE_TO_METERS = LightningMath.in2meters(WHEEL_CIRCUMFERENCE_INCHES)
            / ENCODER_PULSES_PER_REVOLUTION;

}