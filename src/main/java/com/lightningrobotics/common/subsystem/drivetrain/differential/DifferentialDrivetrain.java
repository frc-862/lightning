package com.lightningrobotics.common.subsystem.drivetrain.differential;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.geometry.LightningOdometer;
import com.lightningrobotics.common.geometry.kinematics.*;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;
import com.lightningrobotics.common.util.LightningMath;

import com.lightningrobotics.common.controller.FeedForwardController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DifferentialDrivetrain extends LightningDrivetrain {

    private DifferentialDrivetrainState state = new DifferentialDrivetrainState(0, 0);
    private LightningOdometer odometer;
    private DifferentialGains gains;

    private MotorController[] leftMotors;
    private MotorController[] rightMotors;
    private DoubleSupplier leftVelocity;
    private DoubleSupplier rightVelocity;
    private LightningIMU IMU;
    private DoubleSupplier leftDistance;
    private DoubleSupplier rightDistance;

    private int motorCount = 0;

    ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    SimpleWidget leftEncoderWidget, rightEncoderWidget, navXWidget, rightVel, leftVel;

    public DifferentialDrivetrain(DifferentialGains gains, MotorController[] leftMotors, MotorController[] rightMotors,
            LightningIMU IMU, DoubleSupplier leftVelocity, DoubleSupplier rightVelocity, DoubleSupplier leftDistance,
            DoubleSupplier rightDistance) {

        this.gains = gains;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.IMU = IMU;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;

        this.odometer = new LightningOdometer(IMU.getHeading()); // TODO: use other params

        leftEncoderWidget = tab.add("EncoderLeftVals", ((WPI_TalonFX) leftMotors[0]).getSelectedSensorPosition());
        rightEncoderWidget = tab.add("EncoderRightVals", ((WPI_TalonFX) rightMotors[0]).getSelectedSensorPosition());

        leftVel = tab.add("left speed", 0);
        rightVel = tab.add("right speed", 0);

        navXWidget = tab.add("GyroVals", "");

        configureMotors();
        if (leftMotors.length == rightMotors.length)
            motorCount = leftMotors.length;

        SmartDashboard.putData("Reset everything", new InstantCommand(() -> {
            ((WPI_TalonFX) leftMotors[0]).setSelectedSensorPosition(0);
            ((WPI_TalonFX) rightMotors[0]).setSelectedSensorPosition(0);
            odometer.resetPosition(new Pose2d(), IMU.getHeading());
        }));
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
        state = new DifferentialDrivetrainState(leftVelocity.getAsDouble(), rightVelocity.getAsDouble());
        odometer.update(IMU.getHeading(), leftDistance.getAsDouble(), rightDistance.getAsDouble());

        String gyroData = (odometer.getPoseMeters().getX() + " , " + odometer.getPoseMeters().getY() + " "
                + IMU.getHeading()); // String gyroData = (leftDistance.getAsDouble() + "---" +
                                     // rightDistance.getAsDouble() + " " + IMU.getHeading());

        leftEncoderWidget.withWidget("").getEntry()
                .setNumber(((WPI_TalonFX) leftMotors[0]).getSelectedSensorPosition());
        rightEncoderWidget.withWidget("").getEntry()
                .setNumber(((WPI_TalonFX) rightMotors[0]).getSelectedSensorPosition());

        leftVel.withWidget("").getEntry().setNumber(state.getLeftSpeed());
        rightVel.withWidget("").getEntry().setNumber(state.getRightSpeed());

        navXWidget.withWidget("").getEntry().setString(gyroData);
    }

    public DrivetrainState getDriveTrainState() {
        return state;
    }

    @Override
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public DrivetrainState getDriveState() {
        return state;
    }

    public void resetPose() {
        ((WPI_TalonFX) leftMotors[0]).setSelectedSensorPosition(0);
        ((WPI_TalonFX) rightMotors[0]).setSelectedSensorPosition(0);
        odometer.resetPosition(new Pose2d(), IMU.getHeading());
    }

    public PIDFController getDrivePIDFController() {
        return gains.getPIDFController();
    }

    public FeedForwardController getFeedForwardController() {
        return gains.getFeedForwardController();
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

    public void tankDrive(double leftPWR, double rightPWR) {
        leftPWR = LightningMath.constrain(leftPWR, -1.0, 1.0);
        rightPWR = LightningMath.constrain(rightPWR, -1.0, 1.0);
        setLeftOutput(leftPWR);
        setRightOutput(rightPWR);
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        withEachLeftMotor(m -> m.setVoltage(leftVoltage));
        withEachRightMotor(m -> m.setVoltage(rightVoltage));
    }

    protected void setLeftOutput(double output) {
        withEachLeftMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0))); // withEachLeftMotor(m ->
                                                                                   // m.set(leftDriveController.calculate(LightningMath.constrain(output,
                                                                                   // -1.0, 1.0))));
    }

    protected void setRightOutput(double output) {
        withEachRightMotor(m -> m.set(LightningMath.constrain(output, -1.0, 1.0))); // withEachRightMotor(m ->
                                                                                    // m.set(rightDriveController.calculate(LightningMath.constrain(output,
                                                                                    // -1.0, 1.0))));
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
