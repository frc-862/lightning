package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Represents a single swerve module with both drive and azimuth control.
 */
public class SwerveModule {

    private final SwerveGains gains;
    private final SpeedController driveMotor;
    private final SpeedController angleMotor;
    private final Supplier<Rotation2d> moduleAngle;
    private final DoubleSupplier driveMotorVelocity;
    private final PIDFController driveController;
    private final PIDFController angleController;

    public SwerveModule(SwerveGains gains,
            SpeedController driveMotor,
            SpeedController angleMotor,
            Supplier<Rotation2d> moduleAngle,
            DoubleSupplier driveMotorVelocity,
            PIDFController driveController,
            PIDFController angleController) {

        this.gains = gains;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
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

    public SpeedController getDriveMotor() {
        return driveMotor;
    }

    public SpeedController getAngleMotor() {
        return angleMotor;
    }

    public void setState(SwerveModuleState target) {

        // Optimize the module state
        var state = SwerveModuleState.optimize(target, getModuleAngle());

        // Set drive output
        final var drive = state.velocity / gains.getMaxRealSpeed();
        // final var drive = driveController.calculate(getVelocity(), state.velocity);
        driveMotor.set(drive);

        // Set angle output
        final var angle = angleController.calculate(getModuleAngle().getRadians(), state.angle.getRadians());
        angleMotor.set(angle);

    }

    public void setRawAzimuthPower(double pwr) {
        angleMotor.set(pwr);
    }

    public void setRawDrivePower(double pwr) {
        driveMotor.set(pwr);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getModuleAngle());
    }

}
