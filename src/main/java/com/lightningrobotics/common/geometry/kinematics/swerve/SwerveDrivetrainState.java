package com.lightningrobotics.common.geometry.kinematics.swerve;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;

public class SwerveDrivetrainState extends DrivetrainState {

    private SwerveModuleState[] states;

    public SwerveDrivetrainState(SwerveModuleState[] states) {
        this.states = states;
    }

    public SwerveModuleState[] getStates() {
        return states;
    }

    public void setStates(SwerveModuleState[] states) {
        this.states = states;
    }

    @Override
    public DrivetrainState getState() {
        return this;
    }

}
