// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This file is based on WPILib trajectory.
// https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/wpilibj/trajectory/TrajectoryConfig.java

package com.lightningrobotics.common.geometry.trajectory;

import java.util.ArrayList;
import java.util.List;

import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;

/**
 * Represents the configuration for generating a trajectory. This class stores
 * the start velocity, end velocity, max velocity, max acceleration, custom
 * constraints, and the reversed flag.
 *
 * <p>
 * The class must be constructed with a max velocity and max acceleration. The
 * other parameters (start velocity, end velocity, constraints, reversed) have
 * been defaulted to reasonable values (0, 0, {}, false). These values can be
 * changed via the `setXXX` methods.
 */
public class TrajectoryConfig {

    private final double maxVelocity;

    private final double maxAcceleration;

    private final List<TrajectoryConstraint> constraints;

    private double startVelocity;

    private double endVelocity;

    private boolean reversed;

    /**
     * Constructs the trajectory configuration class.
     * @param drivetrain Drivetrain that will be driving the trajectory.
     * @param reversed If the trajectory should be driven backwards.
     */
    public TrajectoryConfig(LightningDrivetrain drivetrain, boolean reversed) {
        maxVelocity = drivetrain.getGains().getMaxSpeed();
        maxAcceleration = drivetrain.getGains().getMaxAcceleration();
        constraints = new ArrayList<>();
        addConstraint(drivetrain.getConstraint(maxVelocity));
        this.reversed = reversed;
    }

    /**
     * Constructs the trajectory configuration class.
     * @param drivetrain Drivetrain that will be driving the trajectory.
     */
    public TrajectoryConfig(LightningDrivetrain drivetrain) {
        this(drivetrain, false);
    }

    /**
     * Adds a user-defined constraint to the trajectory.
     * @param constraint The user-defined constraint.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig addConstraint(TrajectoryConstraint constraint) {
        constraints.add(constraint);
        return this;
    }

    /**
     * Adds all user-defined constraints from a list to the trajectory.
     * @param constraints List of user-defined constraints.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig addConstraints(List<? extends TrajectoryConstraint> constraints) {
        this.constraints.addAll(constraints);
        return this;
    }

    /**
     * Returns the starting velocity of the trajectory.
     * @return The starting velocity of the trajectory.
     */
    public double getStartVelocity() {
        return startVelocity;
    }

    /**
     * Sets the start velocity of the trajectory.
     * @param startVelocityMetersPerSecond The start velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setStartVelocity(double startVelocityMetersPerSecond) {
        startVelocity = startVelocityMetersPerSecond;
        return this;
    }

    /**
     * Returns the starting velocity of the trajectory.
     * @return The starting velocity of the trajectory.
     */
    public double getEndVelocity() {
        return endVelocity;
    }

    /**
     * Sets the end velocity of the trajectory.
     * @param endVelocityMetersPerSecond The end velocity of the trajectory.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setEndVelocity(double endVelocityMetersPerSecond) {
        endVelocity = endVelocityMetersPerSecond;
        return this;
    }

    /**
     * Returns the maximum velocity of the trajectory.
     * @return The maximum velocity of the trajectory.
     */
    public double getMaxVelocity() {
        return maxVelocity;
    }

    /**
     * Returns the maximum acceleration of the trajectory.
     * @return The maximum acceleration of the trajectory.
     */
    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    /**
     * Returns the user-defined constraints of the trajectory.
     * @return The user-defined constraints of the trajectory.
     */
    public List<TrajectoryConstraint> getConstraints() {
        return constraints;
    }

    /**
     * Returns whether the trajectory is reversed or not.
     * @return whether the trajectory is reversed or not.
     */
    public boolean isReversed() {
        return reversed;
    }

    /**
     * Sets the reversed flag of the trajectory.
     * @param reversed Whether the trajectory should be reversed or not.
     * @return Instance of the current config object.
     */
    public TrajectoryConfig setReversed(boolean reversed) {
        this.reversed = reversed;
        return this;
    }
}
