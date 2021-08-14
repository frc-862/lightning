package com.lightningrobotics.common.geometry;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpiutil.WPIUtilJNI;

public class LightningOdometer {
    
    private final LightningKinematics kinematics;

    private Pose2d pose;
    private double prevTime;

    private Rotation2d headingOffset;
    private Rotation2d previousAngle;

    public LightningOdometer(LightningKinematics kinematics, Rotation2d heading, Pose2d initialPose) {
        this.kinematics = kinematics;
        this.pose = initialPose;
        this.headingOffset = pose.getRotation().minus(heading);
        this.previousAngle = initialPose.getRotation();
    }

    public LightningOdometer(LightningKinematics kinematics, Rotation2d heading) {
        this(kinematics, heading, new Pose2d());
    }
    
    public void reset(Pose2d pose, Rotation2d heading) {
        this.pose = pose;
        this.previousAngle = pose.getRotation();
        this.headingOffset = pose.getRotation().minus(heading);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d update(Rotation2d heading, DrivetrainState state) {
        return updateTime(WPIUtilJNI.now() * 1.0E-6, heading, state);
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

