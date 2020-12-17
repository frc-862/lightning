/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lightning.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lightning.LightningConfig;
import frc.lightning.util.RamseteGains;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

public class CTREDriveTrain extends SubsystemBase implements LightningDrivetrain {
  // private static final double CLOSE_LOOP_RAMP_RATE = 0.5;
  // private static final double OPEN_LOOP_RAMP_RATE = 0.5;

  // DRIVETRAIN
  private final String name = "DRIVETRAIN";

  private BaseMotorController[] leftSlaves;
  private BaseMotorController leftMaster;
  // private CANEncoder leftEncoder;
  // private CANPIDController leftPIDFController;

  private BaseMotorController[] rightSlaves;
  private BaseMotorController rightMaster;
  // private CANEncoder rightEncoder;
  // private CANPIDController rightPIDFController;

  private DifferentialDriveKinematics kinematics;

  private DifferentialDriveOdometry odometry;

  private SimpleMotorFeedforward feedforward;

  private PIDController leftPIDController;

  private PIDController rightPIDController;

  private Pose2d pose = new Pose2d(0d, 0d, new Rotation2d());

  private Pose2d poseOffset = null;

  private RamseteGains gains;

  public CTREDriveTrain(BaseMotorController leftMaster, BaseMotorController rightMaster, BaseMotorController[] leftSlaves,
      BaseMotorController[] rightSlaves, double trackWidth, RamseteGains gains) {
    setName(name);
    this.leftMaster = leftMaster;
    this.rightMaster = rightMaster;
    this.leftSlaves = leftSlaves;
    this.rightSlaves = rightSlaves;

    this.gains = gains;

    configureFollows();

    brake();
    init();

    kinematics = new DifferentialDriveKinematics(trackWidth);

    odometry = new DifferentialDriveOdometry(getHeading(), pose);
    

    feedforward = new SimpleMotorFeedforward(gains.getkS(), gains.getkV(), gains.getkA());

    leftPIDController = new PIDController(gains.getLeft_kP(), gains.getLeft_kI(), gains.getLeft_kD());
        
    rightPIDController = new PIDController(gains.getRight_kP(), gains.getRight_kI(), gains.getRight_kD());
  }

  @Override
  public void periodic(){
    super.periodic();

    SmartDashboard.putNumber("RightMasterHeat", rightMaster.getTemperature());
    SmartDashboard.putNumber("LeftMasterHeat", leftMaster.getTemperature());

    pose = odometry.update(getHeading(), getLeftDistance(), getRightDistance());

  }

  public void init() {
    this.resetDistance();
  }

  protected BaseMotorController getLeftMaster() {
    return leftMaster;
  }

  protected BaseMotorController getRightMaster() {
    return rightMaster;
  }

  protected void withEachMotor(Consumer<BaseMotorController> op) {
    op.accept(leftMaster);
    for (var m : leftSlaves)
      op.accept(m);
    op.accept(rightMaster);
    for (var m : rightSlaves)
      op.accept(m);
  }

  protected void withEachMotorIndexed(BiConsumer<BaseMotorController, Integer> op) {
    op.accept(leftMaster, 0);
    op.accept(rightMaster, 0);
    for (var i = 0; i < leftSlaves.length; ++i) {
      op.accept(leftSlaves[i], i + 1);
    }
    for (var i = 0; i < rightSlaves.length; ++i) {
      op.accept(rightSlaves[i], i + 1);
    }
  }

  protected void withEachSlaveMotor(BiConsumer<BaseMotorController, BaseMotorController> op) {
    for (var m : leftSlaves)
      op.accept(m, leftMaster);
    for (var m : rightSlaves)
      op.accept(m, rightMaster);
  }

  protected void withEachSlaveMotorIndexed(BiConsumer<BaseMotorController, Integer> op) {
    for (var i = 0; i < leftSlaves.length; ++i) {
      op.accept(leftSlaves[i], i + 1);
    }
    for (var i = 0; i < rightSlaves.length; ++i) {
      op.accept(rightSlaves[i], i + 1);
    }
  }

  private void configureFollows() {
    for (var m : leftSlaves)
      m.follow(getLeftMaster());
    for (var m : rightSlaves)
      m.follow(getRightMaster());
  }

  @Override
  public void initMotorDirections() {
  }

  public void setPower(double left, double right) {
    rightMaster.set(ControlMode.PercentOutput, left);
    leftMaster.set(ControlMode.PercentOutput, right);
  }

  public void setVelocity(double left, double right) {
    rightMaster.set(ControlMode.Velocity, left);
    leftMaster.set(ControlMode.Velocity, right);
  }

  public void resetDistance() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public double getLeftDistance() {
    return leftMaster.getSelectedSensorPosition();
  }

  public double getRightDistance() {
    return rightMaster.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return leftMaster.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightMaster.getSelectedSensorVelocity();
  }

  @Override
  public void brake() {
    this.withEachMotor(m -> m.setNeutralMode(NeutralMode.Brake));
  }

  @Override
  public void coast() {
    this.withEachMotor(m -> m.setNeutralMode(NeutralMode.Coast));
  }

  public BaseMotorController[] getLeftMotors() {
    BaseMotorController[] motors = new BaseMotorController[leftSlaves.length + 1];
    motors[0] = getLeftMaster();
    for (int i = 1; i < motors.length; i++)
      motors[i] = leftSlaves[i - 1];
    return motors;
  }

  public BaseMotorController[] getRightMotors() {
    BaseMotorController[] motors = new BaseMotorController[rightSlaves.length + 1];
    motors[0] = getRightMaster();
    for (int i = 1; i < motors.length; i++)
      motors[i] = rightSlaves[i - 1];
    return motors;
  }

  @Override
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public Pose2d getPose() {
    return pose;
  }

  @Override
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  @Override
  public void setOutput(double leftVolts, double rightVolts) {
    SmartDashboard.putNumber("RequestedLeftVolts", leftVolts);
    SmartDashboard.putNumber("RequestedRightVolts", rightVolts);

    leftMaster.set(ControlMode.Velocity, leftVolts);
    rightMaster.set(ControlMode.Velocity, rightVolts);
  }

  @Override
  public PIDController getLeftPidController() {
    return leftPIDController;
  }

  @Override
  public PIDController getRightPidController() {
    return rightPIDController;
  }

  @Override
  public double getRightVolts() {
    return rightMaster.getMotorOutputVoltage() * LightningConfig.VOLT_LIMIT;
  }

  @Override
  public double getLeftVolts() {
    return leftMaster.getMotorOutputVoltage() * LightningConfig.VOLT_LIMIT;
  }

  @Override
  public void setRamseteOutput(double leftVolts, double rightVolts) {
    // TODO Auto-generated method stub

  }

  @Override
  public RamseteGains getConstants() {
    // Override me!
    return null;
  }

  @Override
  public void resetSensorVals() {
    // TODO Auto-generated method stub

  }

  @Override
  public Pose2d getRelativePose() {
    return pose.relativeTo(poseOffset);
  }

  @Override
  public void setRelativePose() {
    poseOffset = pose;
  }
}
