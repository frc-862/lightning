package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import com.lightningrobotics.common.util.LightningMath;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.math.StateSpaceUtil;

public class SwerveModule {
    SpeedController driveController;
    SpeedController angleController;
    SwerveGains gains;

    SwerveModuleState moduleState;

    public SwerveModule(SpeedController driveController, SpeedController angleController, SwerveGains gains){
        this.driveController = driveController;
        this.angleController = angleController;
        this.gains = gains;
    }

    /**
     * Set drive and angular state.
     * @param moduleState 
     * @param currentRotation Current rotation of the robot.
     */
    public void setModuleState(SwerveModuleState moduleState, Rotation2d currentRotation){
        SwerveModuleState state = SwerveModuleState.optimize(moduleState, currentRotation);

        double driveSpeed = LightningMath.constrain(state.speedMetersPerSecond, -gains.getMaxSpeed(), gains.getMaxSpeed());
        double turnSpeed = LightningMath.constrain(state.angle.minus(currentRotation).getDegrees(), 
            -gains.getMaxAngularSpeed(), gains.getMaxAngularSpeed());
            
        double driveOutput = driveSpeed / gains.getMaxSpeed();
        double turnOutput = turnSpeed / gains.getMaxAngularSpeed();

        driveController.set(driveOutput);
        angleController.set(turnOutput);
    }
    public SpeedController getDriveMotor() {
        return driveController;
    }

    public SpeedController getRotationMotor() {
        return angleController;
    }
    
}
