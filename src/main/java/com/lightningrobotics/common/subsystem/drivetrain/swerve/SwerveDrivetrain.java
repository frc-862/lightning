package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.Consumer;

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
    
}
