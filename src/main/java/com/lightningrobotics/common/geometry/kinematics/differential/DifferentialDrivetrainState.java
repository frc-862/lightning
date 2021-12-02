package com.lightningrobotics.common.geometry.kinematics.differential;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;

public class DifferentialDrivetrainState extends DrivetrainState {
    
    private double leftSpeed;
    private double rightSpeed;

    public DifferentialDrivetrainState(double leftSpeed, double rightSpeed) {
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
    }

    public double getLeftSpeed() {
        return leftSpeed;
    }

    public double getRightSpeed() {
        return rightSpeed;
    }

    @Override
    public DrivetrainState getState() {
        // TODO fixme
        return null;
    } 

}
