package com.lightningrobotics.common.geometry;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveDrivetrainState;
import com.lightningrobotics.common.subsystem.core.LightningIMU;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.WPIUtilJNI;

public class LightningOdometer extends SubsystemBase {
    
    private final LightningKinematics kinematics;

    private Pose2d pose;
    private double prevTime;

    private Rotation2d headingOffset;
    private Rotation2d previousAngle;

    private LightningIMU imu;
    private SwerveDrivetrainState state;

    public LightningOdometer(LightningKinematics kinematics, Pose2d initialPose, LightningIMU imu) {
        this.kinematics = kinematics;
        this.pose = initialPose;
        this.headingOffset = pose.getRotation().minus(imu.getHeading());
        this.previousAngle = initialPose.getRotation();
        this.imu = imu;
    }

    public LightningOdometer(LightningKinematics kinematics, LightningIMU imu) {
        this(kinematics, new Pose2d(), imu);
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
        update(state.getState());
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

