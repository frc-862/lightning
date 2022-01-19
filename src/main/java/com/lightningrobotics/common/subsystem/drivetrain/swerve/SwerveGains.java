package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveKinematics;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

public class SwerveGains extends LightningGains {
    
    private SwerveKinematics kinematics;
    private double maxRealSpeed;
    private double maxAngularSpeed;
    private boolean[] turnMotorInverts;
    private boolean[] driveMotorInverts;
    private double width;
    private double length;

    public SwerveGains() {}

    public SwerveGains(double width, double length, double maxSpeed, double maxRealSpeed, double maxAcceleration, double maxAngularSpeed, boolean[] turnMotorInverts, boolean[] driveMotorInverts) {
        super(maxSpeed, maxAcceleration);
        this.width = width;
        this.length = length;
        this.maxRealSpeed = maxRealSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        this.turnMotorInverts = turnMotorInverts;
        this.driveMotorInverts = driveMotorInverts;
        this.kinematics = new SwerveKinematics(this);
        this.setKinematics(kinematics);
    }

    public double getWidth() {
        return width;
    }

    public void setWidth(double width) {
        this.width = width;
    }

    public double getLength() {
        return length;
    }

    public void setLength(double length) {
        this.length = length;
    }

    public void setMaxAngularSpeed(double maxAngularSpeed){
        this.maxAngularSpeed = maxAngularSpeed;
    }

    public double getMaxAngularSpeed(){
        return maxAngularSpeed;
    }

    public boolean[] getTurnMotorInverts() {
        return turnMotorInverts;
    }

    public void setTurnMotorInverts(boolean[] turnMotorInverts) {
        this.turnMotorInverts = turnMotorInverts;
    }

    public boolean[] getDriveMotorInverts() {
        return driveMotorInverts;
    }

    public void setDriveMotorInverts(boolean[] driveMotorInverts) {
        this.driveMotorInverts = driveMotorInverts;
    }

    public double getMaxRealSpeed() {
        return maxRealSpeed;
    }

    public void setMaxRealSpeed(double maxRealSpeed) {
        this.maxRealSpeed = maxRealSpeed;
    }

}
