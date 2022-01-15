package com.lightningrobotics.common.subsystem.drivetrain;

import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;

public class LightningGains {

    public static final double MAX_VOLTAGE = 12d;

    private double maxSpeed;
    private double maxAcceleration;

    public LightningGains() {}

    public LightningGains(double maxSpeed, double maxAcceleration) {
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = maxAcceleration;
    }

    public LightningKinematics getKinematics(){
        return null;
    }

    public double getTrackWidth() {
        return 0;
    }


}
