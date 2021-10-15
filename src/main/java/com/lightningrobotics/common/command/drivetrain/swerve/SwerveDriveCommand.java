package com.lightningrobotics.common.command.drivetrain.swerve;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveGains;
import com.lightningrobotics.common.util.filter.JoystickFilter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {

    private final SwerveDrivetrain drivetrain;
    private final LightningIMU imu;
    private final XboxController controller;
    private final JoystickFilter filter;
    private final boolean fieldCentric;

    public SwerveDriveCommand(SwerveDrivetrain drivetrain, LightningIMU imu, XboxController controller, boolean fieldCentric) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        addRequirements(drivetrain); // we do not add IMU as a requirement because it's use is read-only
        this.controller = controller;
        this.fieldCentric = fieldCentric;
        this.filter = new JoystickFilter(0.05d, 0d, 1d, JoystickFilter.Mode.CUBED);
    }

    @Override
    public void execute() {

        // Get filtered joystick input
        var xInput    = filter.filter(-controller.getY(GenericHID.Hand.kLeft));
        var yInput    = filter.filter(+controller.getX(GenericHID.Hand.kLeft));
        var rotInput  = filter.filter(+controller.getX(GenericHID.Hand.kRight));

        // Constrain magnitude vector from joysticks to w/in practical range
        var magnitude = Math.sqrt((xInput * xInput) + (yInput * yInput));
        if(magnitude > 1d) {
            xInput = xInput / magnitude;
            yInput = yInput / magnitude;
        }

        // Scale joystick input to robot speed
        var gains = (SwerveGains) drivetrain.getGains();
        var xSpeed    =  xInput   * gains.getMaxSpeed();
        var ySpeed    =  yInput   * gains.getMaxSpeed();
        var rotSpeed  =  rotInput * gains.getMaxAngularSpeed();

        // Placeholder for drive speed
        DrivetrainSpeed driveSpeed = null;

        // Convert to field centric if necessary
        if(fieldCentric) {
            final var theta = imu.getHeading();
            driveSpeed = DrivetrainSpeed.fromFieldCentricSpeed(xSpeed, ySpeed, rotSpeed, theta);
        } else {
            driveSpeed = new DrivetrainSpeed(xSpeed, ySpeed, rotSpeed);
        }

        // Set drive speed
        drivetrain.setDriveSpeed(driveSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}
