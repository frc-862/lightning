// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.common.command.drivetrain.differential;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.lightningrobotics.common.auto.trajectory.Trajectory;
import com.lightningrobotics.common.controller.FeedForwardController;
import com.lightningrobotics.common.controller.PIDFController;
import com.lightningrobotics.common.controller.RamseteController;
import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialDrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.differential.DifferentialKinematics;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally. This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard PID
 * functionality of a "smart" motor controller) may use the secondary constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the RAMSETE controller.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */

public class FollowTrajectory extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final FeedForwardController m_feedforward;
  private final DifferentialKinematics m_kinematics;
  private final Supplier<DifferentialDrivetrainState> m_speeds;
  private final PIDFController m_leftController;
  private final PIDFController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDrivetrainState m_prevSpeeds;
  private double m_prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param wheelSpeeds A function that supplies the speeds of the left and right sides of the robot
   *     drive.
   * @param leftController The PIDFController for the left side of the robot drive.
   * @param rightController The PIDFController for the right side of the robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  public FollowTrajectory(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      RamseteController controller,
      FeedForwardController feedforward,
      DifferentialKinematics kinematics,
      Supplier<DifferentialDrivetrainState> wheelSpeeds,
      PIDFController leftController,
      PIDFController rightController,
      BiConsumer<Double, Double> outputVolts, 
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand"); // TODO: change all remseteCommand to followTrajectory
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(controller, "controller", "RamseteCommand");
    m_feedforward = feedforward;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "RamseteCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
    m_output = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");

    addRequirements(requirements);
  }
  
  @Override
  public void initialize() {
    m_prevTime = -1;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds =
        (DifferentialDrivetrainState)m_kinematics.inverse(new DrivetrainSpeed(
          initialState.velocityMetersPerSecond,
          0,
          initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
        ));

    m_timer.reset();
    m_timer.start();
    m_leftController.reset();
    m_rightController.reset();
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    var targetWheelSpeeds =
        m_kinematics.inverse(
            m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = ((DifferentialDrivetrainState)targetWheelSpeeds).getLeftSpeed();
    var rightSpeedSetpoint = ((DifferentialDrivetrainState)targetWheelSpeeds).getRightSpeed();

    double leftOutput;
    double rightOutput;

    double leftFeedforward =
        m_feedforward.calculate(
            leftSpeedSetpoint, (leftSpeedSetpoint - m_prevSpeeds.getLeftSpeed()) / dt);

    double rightFeedforward =
        m_feedforward.calculate(
            rightSpeedSetpoint, (rightSpeedSetpoint - m_prevSpeeds.getRightSpeed()) / dt);

    leftOutput =
        leftFeedforward
            + m_leftController.calculate(m_speeds.get().getLeftSpeed(), leftSpeedSetpoint);

    rightOutput =
        rightFeedforward
            + m_rightController.calculate(
                m_speeds.get().getRightSpeed(), rightSpeedSetpoint);

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = (DifferentialDrivetrainState)targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {    
    m_timer.stop();

    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
