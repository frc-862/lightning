package com.lightningrobotics.common.subsystem.drivetrain.differential;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.LightningKinematics;
import com.lightningrobotics.common.geometry.trajectory.TrajectoryConstraint;
import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;
import com.lightningrobotics.common.util.LightningMath;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class DifferentialDrivetrain extends LightningDrivetrain {

    private static class DifferentialDrivetrainSpeed implements LightningKinematics {

        public double left;
        public double right;

        private DifferentialGains gains;

        public DifferentialDrivetrainSpeed(DrivetrainSpeed speed, DifferentialGains gains) {
            this.gains = gains;
            this.left = speed.vx - gains.getTrackWidth() / 2 * speed.omega;
            this.right = speed.vx + gains.getTrackWidth() / 2 * speed.omega;
            normalize(gains.getMaxSpeed());
        }

        @Override
        public void normalize(double maxSpeed) {
            double realMaxSpeed = Math.max(Math.abs(left), Math.abs(right));
            if (realMaxSpeed > maxSpeed) {
                left /= (realMaxSpeed * maxSpeed);
                right /= (realMaxSpeed * maxSpeed);
            }
        }

        @Override
        public DrivetrainSpeed toDrivetrainSpeed() {
            return new DrivetrainSpeed((left + right) / 2, 0, (right - left) / gains.getTrackWidth());
        }

    }

    /**
     * A class that enforces constraints on the differential drive kinematics. This
     * can be used to ensure that the trajectory is constructed so that the
     * commanded velocities for both sides of the drivetrain stay below a certain
     * limit.
     */
    private class DifferentialDriveTrajectoryConstraint implements TrajectoryConstraint {

        private final double maxSpeedMetersPerSecond;
        
        /**
         * Constructs a differential drive dynamics constraint.
         * @param maxSpeedMetersPerSecond The max speed that a side of the robot can travel at.
         */
        public DifferentialDriveTrajectoryConstraint(double maxSpeedMetersPerSecond) {
            this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        }

        /**
         * Returns the max velocity given the current pose and curvature.
         * @param poseMeters The pose at the current point in the trajectory.
         * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
         * @param velocityMetersPerSecond The velocity at the current point in the trajectory before constraints are applied.
         * @return The absolute maximum velocity.
         */
        @Override
        public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {

            // Create an object to represent the current drivetrain speeds.
            var drivetrainSpeed = new DrivetrainSpeed(velocityMetersPerSecond, 0, velocityMetersPerSecond * curvatureRadPerMeter);

            // Get the wheel speeds and normalize them to within the max velocity.
            var speed = new DifferentialDrivetrainSpeed(drivetrainSpeed, gains);
            speed.normalize(maxSpeedMetersPerSecond);

            // Return the new linear chassis speed.
            return speed.toDrivetrainSpeed().vx;

        }

        /**
         * Returns the minimum and maximum allowable acceleration for the trajectory
         * given pose, curvature, and speed.
         * @param poseMeters The pose at the current point in the trajectory.
         * @param curvatureRadPerMeter The curvature at the current point in the trajectory.
         * @param velocityMetersPerSecond The speed at the current point in the trajectory.
         * @return The min and max acceleration bounds.
         */
        @Override
        public AccelerationLimit getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter, double velocityMetersPerSecond) {
            return new AccelerationLimit();
        }
    }

    private DifferentialGains gains;

    private SpeedController[] leftMotors;
    private SpeedController[] rightMotors;

    private int motorCount = 0;

    public DifferentialDrivetrain(DifferentialGains gains, SpeedController[] leftMotors, SpeedController[] rightMotors) {
        this.gains = gains;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;

        configureMotors();
        if (leftMotors.length == rightMotors.length)
            motorCount = leftMotors.length;
    }

    @Override
    public void configureMotors() {
        var leftInverts = gains.getLeftInverts();
        var rightInverts = gains.getRightInverts();

        if (leftInverts.length == leftMotors.length && rightInverts.length == rightMotors.length) {
            for (var i = 0; i < leftMotors.length; ++i)
                leftMotors[i].setInverted(leftInverts[i]);
            for (var i = 0; i < rightMotors.length; ++i)
                rightMotors[i].setInverted(rightInverts[i]);
        }
    }

    @Override
    public void setDriveSpeed(DrivetrainSpeed speed) {
        var diffSpeed = new DifferentialDrivetrainSpeed(speed, gains);
        var leftOutput = (diffSpeed.left / gains.getMaxSpeed());
        var rightOutput = (diffSpeed.right / gains.getMaxSpeed());

        tankDrive(leftOutput, rightOutput);
    }

    @Override
    public TrajectoryConstraint getConstraint(double maxVelocity) {
        return new DifferentialDriveTrajectoryConstraint(maxVelocity);
    }

    @Override
    public LightningGains getGains() {
        return gains;
    }

    @Override
    public void stop() {
        tankDrive(0.0, 0.0);
    }

    public void arcadeDrive(double speed, double rot) {
        speed = LightningMath.constrain(speed, -1.0, 1.0);
        rot = LightningMath.constrain(rot, -1.0, 1.0);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rot)), speed);

        if (speed >= 0.0) {
            if (rot >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rot;
            } else {
                leftMotorOutput = speed + rot;
                rightMotorOutput = maxInput;
            }
        } else {
            if (rot >= 0.0) {
                leftMotorOutput = speed + rot;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = speed - rot;
            }
        }

        setLeftOutput(leftMotorOutput);
        setRightOutput(rightMotorOutput);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        setLeftOutput(leftSpeed);
        setRightOutput(rightSpeed);
    }

    protected void setLeftOutput(double output) {
        withEachLeftMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0)));
    }

    protected void setRightOutput(double output) {
        withEachRightMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0)));
    }

    protected void withEachMotor(Consumer<SpeedController> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i]);
            op.accept(rightMotors[i]);
        }
    }

    protected void withEachLeftMotor(Consumer<SpeedController> op) {
        for (var i = 0; i < motorCount; ++i)
            op.accept(leftMotors[i]);
    }

    protected void withEachRightMotor(Consumer<SpeedController> op) {
        for (var i = 0; i < motorCount; ++i)
            op.accept(rightMotors[i]);
    }

    protected void withEachMotorIndexed(BiConsumer<SpeedController, Integer> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i], i);
            op.accept(rightMotors[i], i);
        }
    }

    protected void withEachMotor(BiConsumer<SpeedController, SpeedController> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i], leftMotors[0]);
            op.accept(rightMotors[i], rightMotors[0]);
        }
    }

}
