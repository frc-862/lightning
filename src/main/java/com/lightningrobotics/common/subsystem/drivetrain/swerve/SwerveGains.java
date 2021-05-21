package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

public class SwerveGains extends LightningGains {
    
    private double maxAngularSpeed;

    public void setMaxAngularSpeed(double maxAngularSpeed){
        this.maxAngularSpeed = maxAngularSpeed;
    }

    // In degrees
    public double getMaxAngularSpeed(){
        return maxAngularSpeed;
    }
}
