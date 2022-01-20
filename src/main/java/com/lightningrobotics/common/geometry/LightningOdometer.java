package com.lightningrobotics.common.geometry;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;
import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightningOdometer extends SubsystemBase {
    
    private final LightningKinematics kinematics;

    private Pose2d pose;
    private double prevTime;

    private Rotation2d headingOffset;
    private Rotation2d previousAngle;
    private LightningDrivetrain drivetrain;

    private LightningIMU imu;

    public LightningOdometer(LightningKinematics kinematics, Pose2d initialPose, LightningIMU imu, LightningDrivetrain drivetrain) {
        this.kinematics = kinematics;
        this.pose = initialPose;
        this.headingOffset = pose.getRotation().minus(imu.getHeading());
        this.previousAngle = initialPose.getRotation();
        this.imu = imu;
        this.drivetrain = drivetrain;
    }

    public LightningOdometer(LightningKinematics kinematics, LightningIMU imu, LightningDrivetrain drivetrain) {
        this(kinematics, new Pose2d(), imu, drivetrain);
    }
    
    public void reset(Pose2d pose) {
        this.pose = pose;
        this.previousAngle = pose.getRotation();
        this.headingOffset = pose.getRotation().minus(imu.getHeading());
    }

    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void periodic() {
        System.out.println("----------------" + drivetrain.getDriveState() + "=====");
        update(drivetrain.getDriveState());
    }

    public Pose2d update(DrivetrainState state) {
        return updateTime(WPIUtilJNI.now() * 1.0E-6, imu.getHeading(), state);
    }

    public Pose2d updateTime(double currentTime, Rotation2d heading, DrivetrainState state) {

        var elapsed = (prevTime >= 0d) ? currentTime - prevTime : 0d;
        prevTime = currentTime;
        
        var theta = heading.plus(headingOffset);

        var ds = kinematics.forward(state);

        var twist = new Twist2d(ds.vx * elapsed, ds.vy * elapsed, theta.minus(previousAngle).getRadians());
        var next = pose.exp(twist);

        return next;

    }
    
}

