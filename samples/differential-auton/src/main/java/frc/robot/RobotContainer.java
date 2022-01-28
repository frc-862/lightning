package frc.robot;

import java.util.Arrays;

import com.lightningrobotics.common.LightningContainer;
import com.lightningrobotics.common.auto.Autonomous;
import com.lightningrobotics.common.auto.Path;
import com.lightningrobotics.common.command.drivetrain.differential.DifferentialTankDrive;
import com.lightningrobotics.common.subsystem.core.LightningIMU;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.Drivetrain;

public class RobotContainer extends LightningContainer {

	private static final XboxController driver = new XboxController(0);

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
	protected void configureAutonomousCommands() {

		try {
			Autonomous.register("Test Differential Auton 0.5", 
			(new Path(Arrays.asList(new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)), 
				new Pose2d(0.5d, 0d, Rotation2d.fromDegrees(0d))))).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("Test Differential Auton 1", 
			(new Path(Arrays.asList(new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)), 
				new Pose2d(1d, 0d, Rotation2d.fromDegrees(0d))))).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("Test Differential Auton 4", 
			(new Path(Arrays.asList(new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)), 
				new Pose2d(4d, 0d, Rotation2d.fromDegrees(0d))))).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("S-path auton", 
			(new Path(Arrays.asList(new Pose2d(0d, 0d, Rotation2d.fromDegrees(0d)),
				new Pose2d(2d, -1.5d, Rotation2d.fromDegrees(-180d))
				))).getCommand(drivetrain));
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
