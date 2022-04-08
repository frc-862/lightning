package com.lightningrobotics.common.subsystem.core;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base gyroscope type. Supports the {@link com.kauailabs.navx.frc.AHRS NavX} and
 * the {@link com.ctre.phoenix.sensors.PigeonIMU Pigeon}.
 */
public class LightningIMU extends SubsystemBase {

    /**
     * Supported IMU types
     */
    public enum IMUType {
        PIGEON,
        NAVX,
        NONE,
    }

    /**
     * A generic function on the IMU to support lambda structure
     */
    public interface IMUFunction {
        void exec();
    }

    /**
     * Creates a new {@link com.ctre.phoenix.sensors.PigeonIMU Pigeon} with
     * the given ID.
     * @param id CAN ID of the {@link com.ctre.phoenix.sensors.PigeonIMU Pigeon}
     * @return IMU object configured for a {@link com.ctre.phoenix.sensors.PigeonIMU Pigeon}
     */
    public static LightningIMU pigeon(int id) {
        return new LightningIMU(IMUType.PIGEON, id);
    }

    /**
     * Creates a new {@link com.kauailabs.navx.frc.AHRS NavX}.
     * @return IMU object configured for the NavX (SPI)
     */
    public static LightningIMU navX() {
        return new LightningIMU(IMUType.NAVX);
    }

    /**
     * Creates a static IMU
     * @return IMU object that effectively does nothing
     */
    public static LightningIMU none() {
        return new LightningIMU(IMUType.NONE);
    }

    private IMUType type;

    private AHRS navx = null;

    private PigeonIMU pigeon = null;

    private double[] ypr = null;

    private LightningIMU(IMUType type, int id) {
        this.type = type;
        switch (type) {
            case PIGEON:
                pigeon = new PigeonIMU(id);
                ypr = new double[3];
                break;
            case NAVX:
                navx = new AHRS(SPI.Port.kMXP);
                break;
            case NONE:
            default:
                break;
        }

    }

    private LightningIMU(IMUType type) {
        this(type, -1);
    }

    public IMUType getType() {
        return type;
    }


    /**
     * Get the IMU pitch as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}.
     * @return the pitch from the gyro
     */
    public Rotation2d getPitch() {
        if(type == IMUType.NAVX && navx != null) {
            Rotation2d pitch = Rotation2d.fromDegrees(navx.getPitch());
            return pitch;
        }
        if(type == IMUType.PIGEON && ypr != null) {
            
            return Rotation2d.fromDegrees(ypr[1]);
        }
        return Rotation2d.fromDegrees(0d);
    }

    /**
     * Get the IMU heading as a {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d}.
     * @return The heading
     */
    public Rotation2d getHeading() {
        if(type == IMUType.NAVX && navx != null) {
            double heading = navx.getAngle();
            double sign = -Math.signum(heading);
            double filteredRot = sign * (((Math.abs(heading) + 180) % 360) - 180);
            return Rotation2d.fromDegrees(filteredRot);
        }
        if(type == IMUType.PIGEON && ypr != null) {
            double heading = ypr[0];
            double sign = -Math.signum(heading);
            double filteredRot = sign * (((Math.abs(heading) + 180) % 360) - 180);
            return Rotation2d.fromDegrees(filteredRot);
        }
        return Rotation2d.fromDegrees(0d);
    }

    /**
     * A function that can be used to get the heading of the IMU
     * @return A supplier of {@link edu.wpi.first.math.geometry.Rotation2d Rotation2d} objects.
     */
    public Supplier<Rotation2d> heading() {
        return this::getHeading;
    }

    /**
     * Reset IMU heading to 0
     */
    public void reset() {
        if(type == IMUType.NAVX && navx != null) {
            navx.reset();
        }
        if(type == IMUType.PIGEON && pigeon != null) {
            pigeon.setYaw(0d);
            pigeon.setAccumZAngle(0d);
        }
    }

    /**
     * Function to reset IMU heading
     * @return An {@link LightningIMU.IMUFunction} that zeros the IMU heading when called
     */
    public IMUFunction zero() {
        return this::reset;
    }

    public double getNavxAccelerationX(){
        if(type == IMUType.NAVX && navx != null) {
            return navx.getRawAccelX();
        }
        return 0;
    }
    public double getNavxAccelerationY(){
        if(type == IMUType.NAVX && navx != null) {
            return navx.getRawAccelY();
        }
        return 0;
    }
    public double getNavxAccelerationZ(){
        if(type == IMUType.NAVX && navx != null) {
            return navx.getRawAccelZ();
        }
        return 0;
    }

    @Override
    public void periodic() {
        if(type == IMUType.PIGEON && pigeon != null && ypr != null) {
            pigeon.getYawPitchRoll(ypr);
        }
    }

}
