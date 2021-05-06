package com.lightningrobotics.quasar;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.lightningrobotics.common.LightningContainer;
import com.lightningrobotics.common.command.drivetrain.ControllerDrive;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialGains;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;

public class QuasarContainer extends LightningContainer {

    private static final DifferentialGains gains = new DifferentialGains();
    private static final DifferentialDrivetrain drivetrain = new DifferentialDrivetrain(gains, 
        new SpeedController[]{new WPI_TalonFX(RobotMap.LEFT_1_CAN_ID), new WPI_TalonFX(RobotMap.LEFT_2_CAN_ID), new WPI_TalonFX(RobotMap.LEFT_3_CAN_ID)}, 
        new SpeedController[]{new WPI_TalonFX(RobotMap.RIGHT_1_CAN_ID), new WPI_TalonFX(RobotMap.RIGHT_2_CAN_ID), new WPI_TalonFX(RobotMap.RIGHT_3_CAN_ID)});

    private static final Joystick driverLeft = new Joystick(JoystickConstants.DRIVER_LEFT);
    private static final Joystick driverRight = new Joystick(JoystickConstants.DRIVER_RIGHT);

	public QuasarContainer() {
		super();
	}

    @Override
    protected void configureButtonBindings() {

    }

    @Override
    protected void configureSystemTests() {

    }

    @Override
    protected void configureDefaultCommands() {
        drivetrain.setDefaultCommand(ControllerDrive.tank(drivetrain, () -> -driverLeft.getY(), () -> -driverRight.getY()));
    }

    @Override
    protected void releaseDefaultCommands() {

    }

    @Override
    protected void initializeDashboardCommands() {

    }

    @Override
    protected void configureAutonomousCommands() {

    }

    @Override
    protected void configureAutonomousPaths() {
        
    }

    @Override
    protected void configureFaultCodes() {

    }

    @Override
    protected void configureFaultMonitors() {

    }

    @Override
    public LightningDrivetrain getDrivetrain() {
        return drivetrain;
    }
    
}
