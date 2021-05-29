package com.lightningrobotics.common.subsystem.drivetrain.differential;

import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

public class DifferentialGains extends LightningGains {

    private boolean[] leftInverts;
    private boolean[] rightInverts;
    private int trackWidth; 

    public DifferentialGains() {}

    public DifferentialGains(double maxSpeed, double maxAcceleration, int trackWidth, boolean[] leftInverts, boolean[] rightInverts) {
        super(maxSpeed, maxAcceleration);
        this.trackWidth = trackWidth;
        this.leftInverts = leftInverts;
        this.rightInverts = rightInverts;
    }

    public int getTrackWidth() {
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
