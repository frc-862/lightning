package frc.robot;

import com.lightningrobotics.common.LightningContainer;
import com.lightningrobotics.common.command.drivetrain.differential.DifferentialTankDrive;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {

	private static final XboxController driver = new XboxController(0);

	private static final LightningIMU imu = LightningIMU.pigeon(19);

	private static final DifferentialDrivetrain drivetrain = new Drivetrain();

	@Override
	protected void configureButtonBindings() { }

	@Override
	protected void configureSystemTests() { }

	@Override
	protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand(new DifferentialTankDrive(drivetrain, ()-> -driver.getLeftY(), () -> -driver.getRightY()));
	}

	@Override
	protected void releaseDefaultCommands() { }

	@Override
	protected void initializeDashboardCommands() { }

	@Override
	protected void configureAutonomousCommands() { }

	@Override
	protected void configureFaultCodes() { }

	@Override
	protected void configureFaultMonitors() { }

	@Override
	public LightningDrivetrain getDrivetrain() {
		return drivetrain;
	}

}
