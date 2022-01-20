package frc.robot;

import java.util.Arrays;

import com.lightningrobotics.common.LightningContainer;
import com.lightningrobotics.common.auto.Autonomous;
import com.lightningrobotics.common.auto.Path;
import com.lightningrobotics.common.command.drivetrain.swerve.SwerveDriveCommand;
import com.lightningrobotics.common.geometry.LightningOdometer;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {

	private static final XboxController driver = new XboxController(0);

	private static final LightningIMU imu = LightningIMU.pigeon(19);

	private static final SwerveDrivetrain drivetrain = new Drivetrain();

	private static final LightningOdometer lo = new LightningOdometer(drivetrain.getGains().getKinematics(), imu);

	@Override
	protected void configureButtonBindings() { }

	@Override
	protected void configureSystemTests() { }

	@Override
	protected void configureDefaultCommands() {
		drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, imu, driver, true));
	}

	@Override
	protected void releaseDefaultCommands() { }

	@Override
	protected void initializeDashboardCommands() { }

	@Override
	protected void configureAutonomousCommands() {
		try {
			Autonomous.register("Test Swerve Auton", 
			(new Path(Arrays.asList(new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)), 
				new Pose2d(1d, 0d, Rotation2d.fromDegrees(0d))))).getCommand(drivetrain, false));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
	}

	@Override
	protected void configureFaultCodes() { }

	@Override
	protected void configureFaultMonitors() { }

	@Override
	public LightningDrivetrain getDrivetrain() {
		return drivetrain;
	}

}
