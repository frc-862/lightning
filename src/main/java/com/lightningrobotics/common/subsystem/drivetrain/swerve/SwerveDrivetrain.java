package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.trajectory.TrajectoryConstraint;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SpeedController;

public class SwerveDrivetrain extends LightningDrivetrain {

    private SwerveGains gains;

    private SwerveModule[] modules;

    private  SwerveDriveKinematics driveKinematics;

    private LightningIMU imu;

    public SwerveDrivetrain(SwerveGains gains, SwerveDriveKinematics driveKinematics, LightningIMU imu, SwerveModule... modules) {
        this.gains = gains;
        this.modules = modules;
        this.driveKinematics = driveKinematics;
        this.imu = imu;
    }

    @Override
    public void configureMotors() {

    }

    @Override
    public void setDriveSpeed(DrivetrainSpeed speed) {
        Rotation2d currentHeading = imu.getHeading();

        SwerveModuleState[] moduleStates = driveKinematics.toSwerveModuleStates
            (ChassisSpeeds.fromFieldRelativeSpeeds(speed.vx, speed.vy, speed.omega, currentHeading));

        for(int i = 0; i < modules.length; i ++){
            modules[i].setModuleState(moduleStates[i], currentHeading);
        }
    }

    @Override
    public TrajectoryConstraint getConstraint(double maxVelocity) {
        return null;
    }

    @Override
    public LightningGains getGains() {
        return gains;
    }

    @Override
    public void stop() {
        setDriveSpeed(new DrivetrainSpeed(0,0,0));
    }

    public void swerveDriveFieldRelative(){

    }

    protected void withEachModule(Consumer<SwerveModule> op) {
        for (var module : modules)
            op.accept(module);
    }

    protected void withDriveMotor(Consumer<SpeedController> op) {
        for (var module : modules)
            op.accept(module.getDriveMotor());
    }

    protected void withRotationMotor(Consumer<SpeedController> op) {
        for (var module : modules)
            op.accept(module.getRotationMotor());
    }
    
    public void swerveDriveRobotRelative(double xSpeed, double ySpeed, double angularSpeed){
        setDriveSpeed(new DrivetrainSpeed(xSpeed, ySpeed, angularSpeed));
    }
}
