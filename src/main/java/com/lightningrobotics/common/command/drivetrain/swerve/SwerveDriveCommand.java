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

    private static final double DEFAULT_DEADBAND = 0.15;
    private static final double DEFAULT_MIN_PWR = 0.1;
    private static final double DEFAULT_MAX_PWR = 1.0;

    private final SwerveDrivetrain drivetrain;
    private final LightningIMU imu;
    private final XboxController controller;
    private final JoystickFilter filter;
    private final boolean fieldCentric;
    
    public SwerveDriveCommand(SwerveDrivetrain drivetrain, LightningIMU imu, XboxController controller) {
        this(drivetrain, imu, controller, true);
    }

    public SwerveDriveCommand(SwerveDrivetrain drivetrain, LightningIMU imu, XboxController controller, boolean fieldCentric) {
        this(drivetrain, imu, controller, fieldCentric, new JoystickFilter(DEFAULT_DEADBAND, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR, JoystickFilter.Mode.LINEAR));
    }

    public SwerveDriveCommand(SwerveDrivetrain drivetrain, LightningIMU imu, XboxController controller, boolean fieldCentric, JoystickFilter filter) {
        this.drivetrain = drivetrain;
        this.imu = imu;
        addRequirements(drivetrain); // we do not add IMU as a requirement because it's use is read-only
        this.controller = controller;
        this.fieldCentric = fieldCentric;
        this.filter = filter; 
    }

    @Override
    public void execute() {

        // Get filtered joystick input
        var xInput    = filter.filter(-controller.getLeftY());
        var yInput    = filter.filter(+controller.getLeftX());
        var rotInput  = filter.filter(+controller.getRightX());

        // Constrain magnitude vector from joysticks to w/in practical range
        var magnitude = Math.sqrt((xInput * xInput) + (yInput * yInput));
        if(magnitude >= 1d) {
            xInput /= magnitude;
            yInput /= magnitude;
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
