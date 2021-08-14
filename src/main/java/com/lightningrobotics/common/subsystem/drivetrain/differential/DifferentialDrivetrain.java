package com.lightningrobotics.common.subsystem.drivetrain.differential;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.*;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;
import com.lightningrobotics.common.util.LightningMath;

import edu.wpi.first.wpilibj.SpeedController;

public class DifferentialDrivetrain extends LightningDrivetrain {

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
        var state = (DifferentialDrivetrainState) gains.getKinematics().inverse(speed);
        tankDrive(state.getLeftSpeed(), state.getRightSpeed());
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

