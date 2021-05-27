package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveGains extends LightningGains {
    
    private double maxAngularSpeed;
    private boolean[] turnMotorInverts;
    private boolean[] driveMotorInverts;
    private Translation2d[] wheelPositions;

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

}
