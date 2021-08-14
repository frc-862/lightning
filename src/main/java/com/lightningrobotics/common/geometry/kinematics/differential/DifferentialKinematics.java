package com.lightningrobotics.common.geometry.kinematics.differential;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

public class DifferentialKinematics implements LightningKinematics {

    private final DifferentialGains gains;

    public DifferentialKinematics(DifferentialGains gains) {
        this.gains = gains;
    }

    @Override
    public DrivetrainSpeed forward(DrivetrainState state) {
        if(state instanceof DifferentialDrivetrainState) {
            DifferentialDrivetrainState dState = (DifferentialDrivetrainState) state;
            return new DrivetrainSpeed((dState.getLeftSpeed() + dState.getRightSpeed()) / 2, 0, (dState.getRightSpeed() - dState.getLeftSpeed()) / gains.getTrackWidth());
        } else {
            return null;
        }
    }

    @Override
    public DrivetrainState inverse(DrivetrainSpeed speed) {
        var left = speed.vx - gains.getTrackWidth() / 2 * speed.omega;
        var right = speed.vx + gains.getTrackWidth() / 2 * speed.omega;
        return new DifferentialDrivetrainState(left, right);
    }

}

