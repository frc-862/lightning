package com.lightningrobotics.common.command.drivetrain;

import java.util.function.Consumer;

import com.lightningrobotics.common.auto.trajectory.Trajectory;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.geometry.LightningOdometer;
import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.geometry.kinematics.LightningKinematics;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class FollowTrajectory extends CommandBase {

    public FollowTrajectory(
        Trajectory trajectory,
        LightningOdometer pose,
        LightningKinematics kinematics,
        PIDFController xController,
        PIDFController yController,
        PIDFController thetaController,
        Consumer<DrivetrainSpeed> outputSpeed,
        Subsystem... requirements) {
      
    }

    
}
