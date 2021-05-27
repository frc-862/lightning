package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.LightningKinematics;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;
import com.lightningrobotics.common.geometry.trajectory.TrajectoryConstraint;
import com.lightningrobotics.common.subsystem.drivetrain.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SwerveDrivetrain extends LightningDrivetrain {

    private SwerveGains gains;

    private SwerveModule[] modules;

    private static class SwerveDrivetrainSpeed implements LightningKinematics {

        private final SimpleMatrix inverseKinematics;
        private final SimpleMatrix forwardKinematics;

        private final int numModules;
        private final Translation2d[] modules;
        private Translation2d prevCoR = new Translation2d();

        private SwerveModuleState[] moduleStates;

        private SwerveGains gains;

        public SwerveDrivetrainSpeed(DrivetrainSpeed speed, SwerveGains gains, Translation2d centerOfRotationMeters) {
            this.gains = gains;

            numModules = gains.getWheelsDisplacements().length;
            modules = Arrays.copyOf(gains.getWheelsDisplacements(), numModules);
            inverseKinematics = new SimpleMatrix(numModules * 2, 3);

            for (int i = 0; i < numModules; i++) {
                inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -modules[i].getY());
                inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +modules[i].getX());
            }
            forwardKinematics = inverseKinematics.pseudoInverse();

            if (!centerOfRotationMeters.equals(prevCoR)) {
                for (int i = 0; i < numModules; i++) {
                    inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -modules[i].getY() + centerOfRotationMeters.getY());
                    inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +modules[i].getX() - centerOfRotationMeters.getX());
                }
                prevCoR = centerOfRotationMeters;
            }

            var speedVector = new SimpleMatrix(3, 1);
            speedVector.setColumn(0, 0, speed.vx, speed.vy, speed.omega);

            var moduleStatesMatrix = inverseKinematics.mult(speedVector);
            SwerveModuleState[] moduleStates = new SwerveModuleState[numModules];

            for (int i = 0; i < numModules; i++) {
                var x = moduleStatesMatrix.get(i * 2, 0);
                var y = moduleStatesMatrix.get(i * 2 + 1, 0);

                var s = Math.hypot(x, y);
                Rotation2d angle = new Rotation2d(x, y);

                moduleStates[i] = new SwerveModuleState(s, angle);
            }

            this.moduleStates = moduleStates;

            normalize(this.gains.getMaxSpeed());
        }

        public SwerveDrivetrainSpeed(DrivetrainSpeed speed, SwerveGains gains, Translation2d... wheelsMeters) {
            this(speed, gains, new Translation2d());
        }

        @Override
        public void normalize(double maxSpeed) {
            double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
            if (realMaxSpeed > maxSpeed) {
                for (SwerveModuleState moduleState : moduleStates) {
                    moduleState.speedMetersPerSecond = moduleState.speedMetersPerSecond / realMaxSpeed * maxSpeed;
                }
            }
        }

        @Override
        public DrivetrainSpeed toDrivetrainSpeed() {
            var wheelStates = moduleStates;
            var moduleStatesMatrix = new SimpleMatrix(numModules * 2, 1);

            for (int i = 0; i < numModules; i++) {
                var module = wheelStates[i];
                moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
                moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
            }

            var speedVector = forwardKinematics.mult(moduleStatesMatrix);
            return new DrivetrainSpeed(speedVector.get(0, 0), speedVector.get(1, 0), speedVector.get(2, 0));
        }

        public SwerveModuleState[] getStates() {
            return this.moduleStates;
        }

    }

    public SwerveDrivetrain(SwerveGains gains, SwerveModule... modules) {
        this.gains = gains;
        this.modules = modules;
    }

    @Override
    public void configureMotors() {
        var driveInverts = gains.getDriveMotorInverts();
        var turnInverts = gains.getTurnMotorInverts();

        if (driveInverts.length == modules.length && turnInverts.length == modules.length) {
            for (var i = 0; i < modules.length; ++i) {
                modules[i].getDriveMotor().setInverted(driveInverts[i]);
                modules[i].getRotationMotor().setInverted(turnInverts[i]);
            }
        }
    }

    @Override
    public void setDriveSpeed(DrivetrainSpeed speed) {
        var states = new SwerveDrivetrainSpeed(speed, gains).getStates();

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

    public void swerveDriveFieldRelative(double xSpeed, double ySpeed, double rot, Rotation2d currentHeading) {
        setDriveSpeed(DrivetrainSpeed.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, currentHeading));
    }

    public void swerveDriveRobotRelative(double xSpeed, double ySpeed, double angularSpeed) {
        setDriveSpeed(new DrivetrainSpeed(xSpeed, ySpeed, angularSpeed));
    }

    protected void withEachModule(Consumer<SwerveModule> op) {
        for(var module : modules) op.accept(module);
    }

    protected void withEachDriveMotor(Consumer<SpeedController> op) {
        for(var module : modules) op.accept(module.getDriveMotor());
    }

    protected void withEachRotationMotor(Consumer<SpeedController> op) {
        for(var module : modules) op.accept(module.getRotationMotor());
    }

}
