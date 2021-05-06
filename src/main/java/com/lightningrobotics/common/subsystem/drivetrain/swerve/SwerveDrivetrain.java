package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

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
    public LightningGains getGains() {
        return gains;
    }

    @Override
    public void stop() {
        
    }
    
}
