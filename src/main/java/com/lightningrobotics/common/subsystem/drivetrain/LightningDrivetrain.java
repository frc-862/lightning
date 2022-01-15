package com.lightningrobotics.common.subsystem.drivetrain;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LightningDrivetrain extends SubsystemBase {

    public abstract void configureMotors();

    public abstract void setDriveSpeed(DrivetrainSpeed speed);

    public abstract LightningGains getGains();

    public abstract void stop();

    public abstract Pose2d getPose();

}
