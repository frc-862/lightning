package com.lightningrobotics.common.subsystem.drivetrain;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class DrivetrainSpeed {

    public double vx;
    public double vy;
    public double omega;

    public DrivetrainSpeed() {}

    public DrivetrainSpeed(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public static DrivetrainSpeed fromFieldRelativeSpeeds(double vx, double vy, double omega, Rotation2d robotAngle) {
        return new DrivetrainSpeed(vx * robotAngle.getCos() + vy * robotAngle.getSin(), -vx * robotAngle.getSin() + vy * robotAngle.getCos(), omega);
    }

}
