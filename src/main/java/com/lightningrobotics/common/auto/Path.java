package com.lightningrobotics.common.auto;

import java.util.Arrays;
import java.util.List;

import com.lightningrobotics.common.auto.trajectory.Trajectory;
import com.lightningrobotics.common.auto.trajectory.TrajectoryConfig;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Object class representing a path a {@link com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain} can follow.
 */
public class Path {

    /**
     * Name of the path
     */
    private String name;

    /**
     * A list of the path's waypoints
     */
    private List<Pose2d> waypoints;

    /**
     * If the robot is moving forward or backward
     */
    private boolean reversed;

    /**
     * Trajectory of the path
     */
    private Trajectory trajectory;

    /**
     * Constructor creates path object
     * @param waypoints List of waypoints for the optimized path to follow
     */
    public Path(List<Pose2d> waypoints) {
        this("", waypoints, false);
    }

    /**
     * Constructor creates path object
     * @param name The name of the path
     * @param waypoints List of waypoints for the optimized path to follow
     */
    public Path(String name, List<Pose2d> waypoints) {
        this(name, waypoints, false);
    }

    /**
     * Constructor creates path object
     * @param name The name of the path
     * @param waypoints List of waypoints for the optimized path to follow
     * @param reversed Direction robot should follow path
     */
    public Path(String name, List<Pose2d> waypoints, boolean reversed) {
        this.name = name;
        this.waypoints = waypoints;
        this.reversed = reversed;
    }

    /**
     * Name of path
     * @return The name of the path
     */
    public String getName() { return name; }

    /**
     * Direction path should be followed
     * @return The direction the path should be followed
     */
    public boolean getReversed() { return reversed; }

    /**
     * Obtains an optimized trajectory the robot should follow so it hits all the waypoints
     * @param drivetrain Drivetrain object of the robot the path should be configured for
     * @return A trajectory the robot can follow
     */
    protected Trajectory getTrajectory(LightningDrivetrain drivetrain) { 
        if(trajectory != null) return trajectory;

        TrajectoryConfig config = new TrajectoryConfig(drivetrain, getReversed());

        try {
            trajectory = Trajectory.from(waypoints, config);
        } catch (RuntimeException e) {
            System.out.println("ERROR Unable To Generate Trajectory From Path");
            e.printStackTrace();
            trajectory = Trajectory.from(Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(1d, 0d, new Rotation2d())), config);
        }

        return trajectory; 
    }

    /**
     * The duration of time it will take the robot to complete the path
     * @param drivetrain Drivetrain object of the robot the path should be configured for
     * @return The number of seconds the robot will need to complete driving the path
     */
    public double getDuration(LightningDrivetrain drivetrain) {
        if(trajectory != null) return trajectory.getTotalTimeSeconds();
        return this.getTrajectory(drivetrain).getTotalTimeSeconds();
    }

    /**
     * Retrieves the path represented as a command
     * @param drivetrain Drivetrain object of the robot the path should be configured for
     * @return A {@link edu.wpi.first.wpilibj2.command.Command command} representing the path that can be driven by the given drivetrain
     * @throws Exception if given drivetrain is unsupported
     */
    public Command getCommand(LightningDrivetrain drivetrain) throws Exception {
        trajectory = this.getTrajectory(drivetrain);
        if(drivetrain instanceof DifferentialDrivetrain) {
            // some diff drive things
        } else if(drivetrain instanceof SwerveDrivetrain) {

        } else {
            throw new Exception("ERROR: Unsupported Drivetrain Type.\nA drivetrain like no other!");
        }
        //TODO implement
        return null;
    }
    
}