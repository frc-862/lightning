package com.lightningrobotics.common.subsystem.drivetrain.differential;

import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialKinematics;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

public class DifferentialGains extends LightningGains {

    private DifferentialKinematics kinematics;

    private boolean[] leftInverts;
    private boolean[] rightInverts;
    private double trackWidth; 

    public DifferentialGains() {}

    public DifferentialGains(double maxSpeed, double maxAcceleration, double trackWidth, boolean[] leftInverts, boolean[] rightInverts) {
        super(maxSpeed, maxAcceleration);
        this.trackWidth = trackWidth;
        this.leftInverts = leftInverts;
        this.rightInverts = rightInverts;
        this.kinematics = new DifferentialKinematics(this);
        this.setKinematics(kinematics);
    }

    @Override
    public DifferentialKinematics getKinematics() {
        return kinematics;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public void setTrackWidth(int trackWidth) {
        this.trackWidth = trackWidth;
    }

    public boolean[] getLeftInverts() {
        return leftInverts;
    }

    public void setLeftInverts(boolean[] leftInverts) {
        this.leftInverts = leftInverts;
    }

    public boolean[] getRightInverts() {
        return rightInverts;
    }

    public void setRightInverts(boolean[] rightInverts) {
        this.rightInverts = rightInverts;
    }

}
