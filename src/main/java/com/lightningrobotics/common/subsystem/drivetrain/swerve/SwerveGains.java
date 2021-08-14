package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveGains extends LightningGains {
    
    private double maxRealSpeed;
    private double maxAngularSpeed;
    private boolean[] turnMotorInverts;
    private boolean[] driveMotorInverts;
    private Translation2d[] wheelPositions;

    public SwerveGains() {}

    public SwerveGains(double maxSpeed, double maxRealSpeed, double maxAcceleration, double maxAngularSpeed, boolean[] turnMotorInverts, boolean[] driveMotorInverts, Translation2d[] wheelPositions) {
        super(maxSpeed, maxAcceleration);
        this.maxRealSpeed = maxRealSpeed;
        this.maxAngularSpeed = maxAngularSpeed;
        this.turnMotorInverts = turnMotorInverts;
        this.driveMotorInverts = driveMotorInverts;
        this.wheelPositions = wheelPositions;
    }

    public void setMaxAngularSpeed(double maxAngularSpeed){
        this.maxAngularSpeed = maxAngularSpeed;
    }

    // In degrees
    public double getMaxAngularSpeed(){
        return maxAngularSpeed;
    }

    public Translation2d[] getWheelsDisplacements() {
        return wheelPositions;
    }

    public void setWheelsDisplacements(Translation2d... wheelPositions) {
        this.wheelPositions = wheelPositions;
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
