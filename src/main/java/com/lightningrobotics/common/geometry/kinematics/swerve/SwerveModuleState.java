package com.lightningrobotics.common.geometry.kinematics.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveModuleState implements Comparable<SwerveModuleState> {

    public double velocity;
    public Rotation2d angle;

    public SwerveModuleState(double velocity, Rotation2d angle) {
        this.velocity = velocity;
        this.angle = angle;
    }

    public static SwerveModuleState optimize(SwerveModuleState target, Rotation2d currentRotation) {
        var delta = target.angle.minus(currentRotation);
        if(Math.abs(delta.getDegrees()) > 90d) {
            return new SwerveModuleState(-target.velocity, target.angle.rotateBy(Rotation2d.fromDegrees(180d)));
        }
        return target;
    }

    @Override
    public String toString() {
        return "Velocity: " + velocity + " | Angle: " + angle.getDegrees();
    }

    @Override
    public int compareTo(SwerveModuleState o) {
        return Double.compare(this.velocity, o.velocity);
    }

}
