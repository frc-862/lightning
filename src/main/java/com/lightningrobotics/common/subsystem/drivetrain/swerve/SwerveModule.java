package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a single swerve module with both drive and azimuth control.
 */
public class SwerveModule {

    private final SwerveGains gains;
    private final MotorController driveMotor;
    private final MotorController azimuthMotor;
    private final Supplier<Rotation2d> moduleAngle;
    private final DoubleSupplier driveMotorVelocity;
    private final PIDFController driveController;
    private final PIDFController angleController;

    public SwerveModule(SwerveGains gains,
            MotorController driveMotor,
            MotorController angleMotor,
            Supplier<Rotation2d> moduleAngle,
            DoubleSupplier driveMotorVelocity,
            PIDFController driveController,
            PIDFController angleController) {

        this.gains = gains;
        this.driveMotor = driveMotor;
        this.azimuthMotor = angleMotor;
        this.moduleAngle = moduleAngle;
        this.driveMotorVelocity = driveMotorVelocity;
        this.driveController = driveController;
        this.angleController = angleController;

        this.angleController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public Rotation2d getModuleAngle() {
        return moduleAngle.get();
    }

    public double getVelocity() {
        return driveMotorVelocity.getAsDouble();
    }

    public MotorController getDriveMotor() {
        return driveMotor;
    }

    public MotorController getAzimuthMotor() {
        return azimuthMotor;
    }

    public void setState(SwerveModuleState target) {

        // Optimize the module state
        var state = SwerveModuleState.optimize(target, getModuleAngle());

        // Set drive output
        var drive = 0d;
        if (driveController != null) {
            drive = driveController.calculate(getVelocity(), state.velocity);
        } else {
            drive = state.velocity / gains.getMaxRealSpeed();
        }
        driveMotor.set(drive);

        // Set angle output
        final var angle = angleController.calculate(getModuleAngle().getRadians(), state.angle.getRadians());
        azimuthMotor.set(angle);

    }

    public void setRawAzimuthPower(double pwr) {
        azimuthMotor.set(pwr);
    }

    public void setRawDrivePower(double pwr) {
        driveMotor.set(pwr);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getModuleAngle());
    }

}
