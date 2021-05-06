package com.lightningrobotics.common.subsystem.drivetrain;

public class LightningGains {

    public static final double MAX_VOLTAGE = 12d;

    private int maxSpeed;
    private int maxAcceleration;

    public int getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(int maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public int getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(int maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

}
