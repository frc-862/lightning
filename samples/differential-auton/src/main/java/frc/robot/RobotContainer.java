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
			Autonomous.register("1/2 ball path", 
			(new Path("1-2Ball.path", false)).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("3 ball terminal", 
			(new Path("3BallTerminal.path", false)).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("3 ball hanger", 
			(new Path("3BallHanger.path", false)).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("1 meter", 
			(new Path("1Meter.path", false)).getCommand(drivetrain));
		} catch(Exception e) {
			System.err.println("Unexpected Error: " + e.getMessage());
		}
		try {
			Autonomous.register("1 meter forward 1 meter right", 
			(new Path("1Forward1right.path", false)).getCommand(drivetrain));
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
