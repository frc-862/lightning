package com.lightningrobotics.common.subsystem.drivetrain;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.geometry.kinematics.DrivetrainState;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LightningDrivetrain extends SubsystemBase {

    public abstract void configureMotors();

    public abstract void setDriveSpeed(DrivetrainSpeed speed);

    public abstract LightningGains getGains();

    public abstract void stop();

	public SimpleMotorFeedforward getFeedforward() {
		return null;
	}

	public DifferentialDriveKinematics getKinematics() {
		return null;
    }
    
    public SwerveDriveKinematics getSwereKinematics() {
        return null;
    }

    public DrivetrainState getDriveState() {
        return null;
    }
}
