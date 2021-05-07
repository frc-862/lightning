// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// This file is based on WPILib trajectory.
// https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/edu/wpi/first/wpilibj/trajectory/Trajectory.java

package com.lightningrobotics.common.geometry.trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.PoseWithCurvature;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.spline.SplineHelper;
import edu.wpi.first.wpilibj.spline.SplineParameterizer;
import edu.wpi.first.wpilibj.spline.SplineParameterizer.MalformedSplineException;

/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various States that represent the pose, curvature, time elapsed, velocity,
 * and acceleration at that point.
 */
public class Trajectory {

    private final double totalTime;
    private final List<TrajectoryState> trajectoryStates;

    /**
     * Constructs an empty trajectory.
     */
    public Trajectory() {
        trajectoryStates = new ArrayList<>();
        totalTime = 0.0;
    }

    /**
     * Constructs a trajectory from a vector of states.
     * @param states A vector of states.
     */
    public Trajectory(final List<TrajectoryState> states) {
        this.trajectoryStates = states;
        totalTime = states.get(states.size() - 1).timeSeconds;
    }

    /**
     * Generates a trajectory from the given waypoints and config. This method uses
     * quintic hermite splines -- therefore, all points must be represented by
     * Pose2d objects. Continuous curvature is guaranteed in this method.
     * @param waypoints List of waypoints..
     * @param config    The configuration for the trajectory.
     * @return The generated trajectory.
     */
    public static Trajectory from(List<Pose2d> waypoints, TrajectoryConfig config) {
        List<Pose2d> newWaypoints = new ArrayList<>();

        // Make list of new waypoints with flip if reversed
        final var flip = new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180.0));
        if (config.isReversed()) {
            for (Pose2d originalWaypoint : waypoints) {
                newWaypoints.add(originalWaypoint.plus(flip));
            }
        } else {
            newWaypoints.addAll(waypoints);
        }

        // Get the spline points
        List<PoseWithCurvature> points;
        try {
            points = splinePointsFromSplines(SplineHelper.getQuinticSplinesFromWaypoints(newWaypoints));
        } catch (MalformedSplineException ex) {
            return null;
        }

        // Change the points back to their original orientation.
        if (config.isReversed()) {
            for (var point : points) {
                point.poseMeters = point.poseMeters.plus(flip);
                point.curvatureRadPerMeter *= -1;
            }
        }

        // Generate and return trajectory.
        return TrajectoryInterpolater.timeParameterizeTrajectory(points, config.getConstraints(),
                config.getStartVelocity(), config.getEndVelocity(), config.getMaxVelocity(),
                config.getMaxAcceleration(), config.isReversed());
    }

    /**
     * Generate spline points from a vector of splines by parameterizing the splines.
     * @param splines The splines to parameterize.
     * @return The spline points for use in time parameterization of a trajectory.
     * @throws MalformedSplineException When the spline is malformed (e.g. has close
     *                                  adjacent points with approximately opposing
     *                                  headings)
     */
    private static List<PoseWithCurvature> splinePointsFromSplines(Spline[] splines) {
        // Create the vector of spline points.
        var splinePoints = new ArrayList<PoseWithCurvature>();

        // Add the first point to the vector.
        splinePoints.add(splines[0].getPoint(0.0));

        // Iterate through the vector and parameterize each spline, adding the
        // parameterized points to the final vector.
        for (final var spline : splines) {
            var points = SplineParameterizer.parameterize(spline);

            // Append the array of poses to the vector. We are removing the first
            // point because it's a duplicate of the last point from the previous
            // spline.
            splinePoints.addAll(points.subList(1, points.size()));
        }
        return splinePoints;
    }

    /**
     * Returns the initial pose of the trajectory.
     * 
     * @return The initial pose of the trajectory.
     */
    public Pose2d getInitialPose() {
        return sample(0).poseMeters;
    }

    /**
     * Returns the overall duration of the trajectory.
     * 
     * @return The duration of the trajectory.
     */
    public double getTotalTimeSeconds() {
        return totalTime;
    }

    /**
     * Return the states of the trajectory.
     * 
     * @return The states of the trajectory.
     */
    public List<TrajectoryState> getStates() {
        return trajectoryStates;
    }

    /**
     * Sample the trajectory at a point in time.
     * 
     * @param timeSeconds The point in time since the beginning of the trajectory to
     *                    sample.
     * @return The state at that point in time.
     */
    public TrajectoryState sample(double timeSeconds) {
        if (timeSeconds <= trajectoryStates.get(0).timeSeconds) {
            return trajectoryStates.get(0);
        }
        if (timeSeconds >= totalTime) {
            return trajectoryStates.get(trajectoryStates.size() - 1);
        }

        // To get the element that we want, we will use a binary search algorithm
        // instead of iterating over a for-loop. A binary search is O(std::log(n))
        // whereas searching using a loop is O(n).

        // This starts at 1 because we use the previous state later on for
        // interpolation.
        int low = 1;
        int high = trajectoryStates.size() - 1;

        while (low != high) {
            int mid = (low + high) / 2;
            if (trajectoryStates.get(mid).timeSeconds < timeSeconds) {
                // This index and everything under it are less than the requested
                // timestamp. Therefore, we can discard them.
                low = mid + 1;
            } else {
                // t is at least as large as the element at this index. This means that
                // anything after it cannot be what we are looking for.
                high = mid;
            }
        }

        // High and Low should be the same.

        // The sample's timestamp is now greater than or equal to the requested
        // timestamp. If it is greater, we need to interpolate between the
        // previous state and the current state to get the exact state that we
        // want.
        final TrajectoryState sample = trajectoryStates.get(low);
        final TrajectoryState prevSample = trajectoryStates.get(low - 1);

        // If the difference in states is negligible, then we are spot on!
        if (Math.abs(sample.timeSeconds - prevSample.timeSeconds) < 1E-9) {
            return sample;
        }
        // Interpolate between the two states for the state that we want.
        return prevSample.interpolate(sample,
                (timeSeconds - prevSample.timeSeconds) / (sample.timeSeconds - prevSample.timeSeconds));
    }

    /**
     * Transforms all poses in the trajectory by the given transform. This is useful
     * for converting a robot-relative trajectory into a field-relative trajectory.
     * This works with respect to the first pose in the trajectory.
     * 
     * @param transform The transform to transform the trajectory by.
     * @return The transformed trajectory.
     */
    public Trajectory transformBy(Transform2d transform) {
        var firstState = trajectoryStates.get(0);
        var firstPose = firstState.poseMeters;

        // Calculate the transformed first pose.
        var newFirstPose = firstPose.plus(transform);
        List<TrajectoryState> newStates = new ArrayList<>();

        newStates.add(new TrajectoryState(firstState.timeSeconds, firstState.velocityMetersPerSecond,
                firstState.accelerationMetersPerSecondSq, newFirstPose, firstState.curvatureRadPerMeter));

        for (int i = 1; i < trajectoryStates.size(); i++) {
            var state = trajectoryStates.get(i);
            // We are transforming relative to the coordinate frame of the new initial pose.
            newStates.add(new TrajectoryState(state.timeSeconds, state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq, newFirstPose.plus(state.poseMeters.minus(firstPose)),
                    state.curvatureRadPerMeter));
        }

        return new Trajectory(newStates);
    }

    /**
     * Transforms all poses in the trajectory so that they are relative to the given
     * pose. This is useful for converting a field-relative trajectory into a
     * robot-relative trajectory.
     * 
     * @param pose The pose that is the origin of the coordinate frame that the
     *             current trajectory will be transformed into.
     * @return The transformed trajectory.
     */
    public Trajectory relativeTo(Pose2d pose) {
        return new Trajectory(trajectoryStates.stream()
                .map(state -> new TrajectoryState(state.timeSeconds, state.velocityMetersPerSecond,
                        state.accelerationMetersPerSecondSq, state.poseMeters.relativeTo(pose),
                        state.curvatureRadPerMeter))
                .collect(Collectors.toList()));
    }

    /**
     * Concatenates another trajectory to the current trajectory. The user is
     * responsible for making sure that the end pose of this trajectory and the
     * start pose of the other trajectory match (if that is the desired behavior).
     * 
     * @param other The trajectory to concatenate.
     * @return The concatenated trajectory.
     */
    public Trajectory concatenate(Trajectory other) {
        // If this is a default constructed trajectory with no states, then we can
        // simply return the rhs trajectory.
        if (trajectoryStates.isEmpty()) {
            return other;
        }

        // Deep copy the current states.
        List<TrajectoryState> states = trajectoryStates.stream()
                .map(state -> new TrajectoryState(state.timeSeconds, state.velocityMetersPerSecond,
                        state.accelerationMetersPerSecondSq, state.poseMeters, state.curvatureRadPerMeter))
                .collect(Collectors.toList());

        // Here we omit the first state of the other trajectory because we don't want
        // two time points with different states. Sample() will automatically
        // interpolate between the end of this trajectory and the second state of the
        // other trajectory.
        for (int i = 1; i < other.getStates().size(); ++i) {
            var s = other.getStates().get(i);
            states.add(new TrajectoryState(s.timeSeconds + totalTime, s.velocityMetersPerSecond,
                    s.accelerationMetersPerSecondSq, s.poseMeters, s.curvatureRadPerMeter));
        }
        return new Trajectory(states);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Trajectory && trajectoryStates.equals(((Trajectory) obj).getStates());
    }

}
