package com.lightningrobotics.common.subsystem.drivetrain.differential;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.geometry.LightningOdometer;
import com.lightningrobotics.common.geometry.kinematics.*;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;
import com.lightningrobotics.common.util.LightningMath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class DifferentialDrivetrain extends LightningDrivetrain {

    private DifferentialDrivetrainState state;
    private LightningOdometer Odometer;
    private LightningIMU IMU = LightningIMU.pigeon(19);

    private DifferentialGains gains;

    private PIDFController

    private MotorController[] leftMotors;
    private MotorController[] rightMotors;

    private int motorCount = 0;

    public DifferentialDrivetrain(DifferentialGains gains, MotorController[] leftMotors, MotorController[] rightMotors) {
        this.gains = gains;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.Odometer = new LightningOdometer(gains.getKinematics(), IMU.getHeading());

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
        state = (DifferentialDrivetrainState) gains.getKinematics().inverse(speed);
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

    @Override
    public void periodic() {
        Odometer.update(IMU.getHeading(), state);
    }

    @Override
    public Pose2d getPose(){
        return Odometer.getPose();
    }

    public DifferentialDrivetrainState getDrivetrainState(){
        return state;
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
        leftSpeed = LightningMath.constrain(leftSpeed, -1.0, 1.0);
        rightSpeed = LightningMath.constrain(rightSpeed, -1.0, 1.0);
        setLeftOutput(leftSpeed);
        setRightOutput(rightSpeed);
    }

    public void setVoltage(double leftVoltage, double rightVoltage){
        withEachLeftMotor(m -> m.setVoltage(leftVoltage));
        withEachRightMotor(m -> m.setVoltage(rightVoltage));
    }

    protected void setLeftOutput(double output) {
        withEachLeftMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0)));
    }

    protected void setRightOutput(double output) {
        withEachRightMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0)));
    }

    protected void withEachMotor(Consumer<MotorController> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i]);
            op.accept(rightMotors[i]);
        }
    }

    protected void withEachLeftMotor(Consumer<MotorController> op) {
        for (var i = 0; i < motorCount; ++i)
            op.accept(leftMotors[i]);
    }

    protected void withEachRightMotor(Consumer<MotorController> op) {
        for (var i = 0; i < motorCount; ++i)
            op.accept(rightMotors[i]);
    }

    protected void withEachMotorIndexed(BiConsumer<MotorController, Integer> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i], i);
            op.accept(rightMotors[i], i);
        }
    }

    protected void withEachMotor(BiConsumer<MotorController, MotorController> op) {
        for (var i = 0; i < motorCount; ++i) {
            op.accept(leftMotors[i], leftMotors[0]);
            op.accept(rightMotors[i], rightMotors[0]);
        }
    }

}

