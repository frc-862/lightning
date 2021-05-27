package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;
import com.lightningrobotics.common.geometry.trajectory.TrajectoryConstraint;
import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import edu.wpi.first.wpilibj.SpeedController;

public class SwerveDrivetrain extends LightningDrivetrain {

    private SwerveGains gains;

    private SwerveModule[] modules;

    public SwerveDrivetrain(SwerveGains gains, SwerveModule... modules) {
        this.gains = gains;
        this.modules = modules;
    }

    @Override
    public void configureMotors() {
        
    }

    @Override
    public void setDriveSpeed(DrivetrainSpeed speed) {

        // TODO DrivetrainSpeed to SwerveModuleState

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];
            module.setDesiredState(state);
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
        this.setDriveSpeed(new DrivetrainSpeed(0d, 0d, 0d));
    }

    public void swerveDriveFieldRelative(){

    }

    public void swerveDriveRobotRelative(double xSpeed, double ySpeed, double angularSpeed){
        setDriveSpeed(new DrivetrainSpeed(xSpeed, ySpeed, angularSpeed));
    }

    protected void withEachModule(Consumer<SwerveModule> op) {
        for (var module : modules)
            op.accept(module);
    }

    protected void withEachDriveMotor(Consumer<SpeedController> op) {
        for (var module : modules)
            op.accept(module.getDriveMotor());
    }

    protected void withEachRotationMotor(Consumer<SpeedController> op) {
        for (var module : modules)
            op.accept(module.getRotationMotor());
    }
    
}
