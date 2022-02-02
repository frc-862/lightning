package com.lightningrobotics.common.auto;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiConsumer;

import com.lightningrobotics.common.auto.trajectory.Trajectory;
import com.lightningrobotics.common.auto.trajectory.TrajectoryConfig;
import com.lightningrobotics.common.command.drivetrain.differential.FollowTrajectory;
import com.lightningrobotics.common.controller.DiffDriveController;
import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialKinematics;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.io.*;
import java.lang.Math;
import java.nio.file.Paths;
import java.util.Scanner;

/**
 * Object class representing a path a
 * {@link com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain}
 * can follow.
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
     * 
     * @param waypoints List of waypoints for the optimized path to follow
     */
    public Path(List<Pose2d> waypoints) {
        this("", waypoints, false);
    }

    public Path(List<Pose2d> waypoints, boolean reversed) {
        this("", waypoints, reversed);
    }

    /**
     * Constructor creates path object
     * 
     * @param name      The name of the path
     * @param waypoints List of waypoints for the optimized path to follow
     */
    public Path(String name, List<Pose2d> waypoints) {
        this(name, waypoints, false);
    }

    /**
     * Constructor creates path object
     * 
     * @param name      The name of the path
     * @param waypoints List of waypoints for the optimized path to follow
     * @param reversed  Direction robot should follow path
     */
    public Path(String name, List<Pose2d> waypoints, boolean reversed) {
        this.name = name;
        this.waypoints = waypoints;
        this.reversed = reversed;
    }

    public Path(String fname, boolean reversed) {

        List<Pose2d> waypoints = new ArrayList<Pose2d>();

        File file = Paths.get(Filesystem.getDeployDirectory().getAbsolutePath(), "paths", fname).toFile();

        try {
            double x_prime;
            double y_prime;

            double max_y = 8.229;

            Scanner in = new Scanner(file);
            in.useDelimiter(",");
            in.nextLine(); // Skip first line with "X Y Theta"

            double start_x = in.nextDouble();
            double start_y = max_y + in.nextDouble();
            double start_theta_x = in.nextDouble();
            double start_theta_y = in.nextDouble();
            double start_theta = Math.toDegrees(Math.atan2(start_theta_y, start_theta_x));

            double rotaional_theta = start_theta;

            in.nextLine();
            waypoints.add(new Pose2d((start_x - start_x), (start_y - start_y),
                    Rotation2d.fromDegrees((start_theta - start_theta))));

            while (in.hasNextDouble()) {
                double x = in.nextDouble() - start_x;
                double y = (max_y + in.nextDouble()) - start_y;

                x_prime = (x * Math.cos(Math.toRadians(rotaional_theta)))
                        + (y * Math.sin(Math.toRadians(rotaional_theta)));
                y_prime = -(x * Math.sin(Math.toRadians(rotaional_theta)))
                        + (y * Math.cos(Math.toRadians(rotaional_theta)));

                if (reversed == true) { // TODO: check if these really need to be sign filpped
                    y = -y;
                    x = -x;
                }

                double theta_x = in.nextDouble();
                double theta_y = in.nextDouble();
                double theta = Math.toDegrees(Math.atan2(theta_y, theta_x)) - start_theta;

                if (theta < -180) {
                    theta = theta + 360;
                } else if (theta > 180) {
                    theta = theta - 360;
                }

                waypoints.add(new Pose2d(x_prime, y_prime, Rotation2d.fromDegrees(theta)));

                in.nextLine();
            }
            in.close();
        } catch (Exception e) {
            System.err.println("COULD NOT READ PATH");
            e.printStackTrace();
        }

        this.waypoints = waypoints;
        this.reversed = reversed;
        this.name = "";
    }

    /**
     * Name of path
     * 
     * @return The name of the path
     */
    public String getName() {
        return name;
    }

    /**
     * Direction path should be followed
     * 
     * @return The direction the path should be followed
     */
    public boolean getReversed() {
        return reversed;
    }

    /**
     * Obtains an optimized trajectory the robot should follow so it hits all the
     * waypoints
     * 
     * @param <TrajectoryConfig>
     * @param drivetrain         Drivetrain object of the robot the path should be
     *                           configured for
     * @return A trajectory the robot can follow
     */
    public Trajectory getTrajectory(LightningDrivetrain drivetrain) {
        if (trajectory != null)
            return trajectory;

        TrajectoryConfig config = new TrajectoryConfig(drivetrain, getReversed());

        try {
            trajectory = Trajectory.from(waypoints, config);
        } catch (RuntimeException e) {
            System.out.println("ERROR Unable To Generate Trajectory From Path");
            e.printStackTrace();
            trajectory = Trajectory.from(
                    Arrays.asList(new Pose2d(0d, 0d, new Rotation2d()), new Pose2d(1d, 0d, new Rotation2d())), config);
        }

        return trajectory;
    }

    /**
     * The duration of time it will take the robot to complete the path
     * 
     * @param drivetrain Drivetrain object of the robot the path should be
     *                   configured for
     * @return The number of seconds the robot will need to complete driving the
     *         path
     */
    public double getDuration(LightningDrivetrain drivetrain) {
        if (trajectory != null)
            return trajectory.getTotalTimeSeconds();
        return this.getTrajectory(drivetrain).getTotalTimeSeconds();
    }

    /**
     * Retrieves the path represented as a command
     * 
     * @param drivetrain Drivetrain object of the robot the path should be
     *                   configured for
     * @return A {@link edu.wpi.first.wpilibj2.command.Command command} representing
     *         the path that can be driven by the given drivetrain
     * @throws Exception if given drivetrain is unsupported
     */
    public Command getCommand(LightningDrivetrain drivetrain) throws Exception {
        trajectory = this.getTrajectory(drivetrain);
        if (drivetrain instanceof DifferentialDrivetrain) {
            DifferentialDrivetrain differentialDrivetrain = (DifferentialDrivetrain) drivetrain;
            BiConsumer<Double, Double> voltageConsumer = (l, r) -> ((DifferentialDrivetrain) drivetrain).setVoltage(l, r);

            return new FollowTrajectory(trajectory,
                    drivetrain::getPose,
                    new DiffDriveController(),
                    differentialDrivetrain.getFeedForwardController(),
                    (DifferentialKinematics) drivetrain.getGains().getKinematics(),
                    () -> (DifferentialDrivetrainState) differentialDrivetrain.getDriveState(),
                    differentialDrivetrain.getDrivePIDFController(),
                    differentialDrivetrain.getDrivePIDFController(),
                    voltageConsumer,
                    drivetrain) {
                @Override
                public void initialize() {
                    super.initialize();
                    differentialDrivetrain.resetPose();
                };
            };

        } else if (drivetrain instanceof SwerveDrivetrain) {
            throw new Exception("ERROR: We dont do swerves yet");
        } else {
            throw new Exception("ERROR: Unsupported Drivetrain Type.\nA drivetrain like no other!");
        }
    }

}