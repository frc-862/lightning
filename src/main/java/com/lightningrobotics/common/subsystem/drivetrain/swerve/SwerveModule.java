package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.DoubleSupplier;

import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class SwerveModule {

    SwerveGains gains;

    SpeedController driveMotor;
    SpeedController angleMotor;
    DoubleSupplier angle;

    SwerveModuleState moduleState;

    public SwerveModule(SwerveGains gains, SpeedController driveController, SpeedController angleController, DoubleSupplier angle) {
        this.gains = gains;
        this.driveMotor = driveController;
        this.angleMotor = angleController;
        this.angle = angle;
    }

    /**
     * Set drive and angular state.
     * @param desiredState Target state to move motors to.
     */
    public void setDesiredState(SwerveModuleState desiredState){

        Rotation2d currentRotation = getAngle();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // TODO use some kind of closed loop control

        driveMotor.set(state.speedMetersPerSecond / gains.getMaxSpeed());

        // final double turnOutput = turnController.calculate(currentRotation.getRadians(), state.angle.getRadians());
        // final double turnFeedForward = turnFF.calculate(turnController.getSetpoint().velocity);
        // angleMotor.setVoltage(turnOutput + turnFeedForward);

    }

    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angle.getAsDouble()); 
    }
    
    public SpeedController getDriveMotor() {
        return driveMotor;
    }

    public SpeedController getRotationMotor() {
        return angleMotor;
    }
    
}
