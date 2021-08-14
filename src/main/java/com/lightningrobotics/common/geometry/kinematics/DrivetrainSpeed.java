package com.lightningrobotics.common.geometry.kinematics;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Class represents a speed vector for the robot, relative to the robot.
 */
public class DrivetrainSpeed {

    /**
     * Velocity in X direction
     */
    public final double vx;

    /**
     * Velocity in Y direction
     */
    public final double vy;

    /**
     * Rotational velocity
     */
    public final double omega;

    /**
     * Creates a new, zero drivetrain speed.
     */
    public DrivetrainSpeed() {
        this.vx     = 0d;
        this.vy     = 0d;
        this.omega  = 0d;
    }

    /**
     * Creates a new drivetrain speed.
     * @param vx Velocity in X direction
     * @param vy Velocity in Y direction
     * @param omega Rotational velocity
     */
    public DrivetrainSpeed(double vx, double vy, double omega) {
        this.vx     = vx;
        this.vy     = vy;
        this.omega  = omega;
    }

    /**
     * Creates a drivetrain speed from a field-centric velocity.
     * @param vx Velocity in X direction relative to the field
     * @param vy Velocity in Y direction relative to the field
     * @param omega Rotational velocity relative to the field
     * @param theta Robot's orientation relative to the field
     * @return A new drivetrain speed equal to the input speed relative to the robot.
     */
    public static DrivetrainSpeed fromFieldCentricSpeed(double vx, double vy, double omega, Rotation2d theta) {
        // Rotate vector <xSpeed, ySpeed> by theta
        var xSpeed =  (vx * theta.getCos()) + (vy * theta.getSin());
        var ySpeed = -(vx * theta.getSin()) + (vy * theta.getCos());
        return new DrivetrainSpeed(xSpeed, ySpeed, omega);
    }

    @Override
    public String toString() {
        return "X: " + vx + " | Y: " + vy + " | ROT: " + omega;
    }

}
