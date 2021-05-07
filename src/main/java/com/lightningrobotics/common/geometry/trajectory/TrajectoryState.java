// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This file is based on WPILib trajectory.
// https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/wpilibj/trajectory/Trajectory.java

package com.lightningrobotics.common.geometry.trajectory;

import java.util.Objects;

import com.lightningrobotics.common.util.LightningMath;

import edu.wpi.first.wpilibj.geometry.Pose2d;

/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various TrajectoryStates that represent the pose, curvature, time elapsed,
 * velocity, and acceleration at that point.
 */
public class TrajectoryState {

    /**
     * The time elapsed since the beginning of the trajectory.
     */
    public double timeSeconds;

    /**
     * The speed at that point of the trajectory.
     */
    public double velocityMetersPerSecond;

    /**
     * The acceleration at that point of the trajectory.
     */
    public double accelerationMetersPerSecondSq;

    /**
     * The pose at that point of the trajectory.
     */
    public Pose2d poseMeters;

    /**
     * The curvature at that point of the trajectory.
     */
    public double curvatureRadPerMeter;

    public TrajectoryState() {
        poseMeters = new Pose2d();
    }

    /**
     * Constructs a TrajectoryState with the specified parameters.
     * @param timeSeconds The time elapsed since the beginning of the trajectory.
     * @param velocityMetersPerSecond The speed at that point of the trajectory.
     * @param accelerationMetersPerSecondSq The acceleration at that point of the trajectory.
     * @param poseMeters The pose at that point of the trajectory.
     * @param curvatureRadPerMeter The curvature at that point of the trajectory.
     */
    public TrajectoryState(double timeSeconds, double velocityMetersPerSecond, double accelerationMetersPerSecondSq,
            Pose2d poseMeters, double curvatureRadPerMeter) {
        this.timeSeconds = timeSeconds;
        this.velocityMetersPerSecond = velocityMetersPerSecond;
        this.accelerationMetersPerSecondSq = accelerationMetersPerSecondSq;
        this.poseMeters = poseMeters;
        this.curvatureRadPerMeter = curvatureRadPerMeter;
    }

    /**
     * Interpolates between two TrajectoryStates.
     * @param endValue The end value for the interpolation.
     * @param i        The interpolant (fraction).
     * @return The interpolated TrajectoryState.
     */
    TrajectoryState interpolate(TrajectoryState endValue, double i) {
        // Find the new t value.
        final double newT = LightningMath.lerp(timeSeconds, endValue.timeSeconds, i);

        // Find the delta time between the current TrajectoryState and the interpolated
        // TrajectoryState.
        final double deltaT = newT - timeSeconds;

        // If delta time is negative, flip the order of interpolation.
        if (deltaT < 0) {
            return endValue.interpolate(this, 1 - i);
        }

        // Check whether the robot is reversing at this stage.
        final boolean reversing = velocityMetersPerSecond < 0
                || Math.abs(velocityMetersPerSecond) < 1E-9 && accelerationMetersPerSecondSq < 0;

        // Calculate the new velocity
        // v_f = v_0 + at
        final double newV = velocityMetersPerSecond + (accelerationMetersPerSecondSq * deltaT);

        // Calculate the change in position.
        // delta_s = v_0 t + 0.5 at^2
        final double newS = (velocityMetersPerSecond * deltaT
                + 0.5 * accelerationMetersPerSecondSq * Math.pow(deltaT, 2)) * (reversing ? -1.0 : 1.0);

        // Return the new TrajectoryState. To find the new position for the new
        // TrajectoryState, we need
        // to interpolate between the two endpoint poses. The fraction for
        // interpolation is the change in position (delta s) divided by the total
        // distance between the two endpoints.
        final double interpolationFrac = newS
                / endValue.poseMeters.getTranslation().getDistance(poseMeters.getTranslation());

        return new TrajectoryState(newT, newV, accelerationMetersPerSecondSq,
                LightningMath.lerp(poseMeters, endValue.poseMeters, interpolationFrac),
                LightningMath.lerp(curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac));
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof TrajectoryState)) {
            return false;
        }
        TrajectoryState state = (TrajectoryState) obj;
        return Double.compare(state.timeSeconds, timeSeconds) == 0
                && Double.compare(state.velocityMetersPerSecond, velocityMetersPerSecond) == 0
                && Double.compare(state.accelerationMetersPerSecondSq, accelerationMetersPerSecondSq) == 0
                && Double.compare(state.curvatureRadPerMeter, curvatureRadPerMeter) == 0
                && Objects.equals(poseMeters, state.poseMeters);
    }

}
