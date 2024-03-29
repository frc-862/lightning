package com.lightningrobotics.common.command.core;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.Map;

/**
 * Command that waits for a dashboard-determined amount of time to elapse before returning.
 * This is useful for adding tunable delays to auton sequences to better-coordinate with alliance partners.
 */
public class WaitCommand extends CommandBase {
	private NetworkTableEntry entry;
	private double targetTime;

	/**
	 * Creates a new DashboardWaitCommand called {@code AutoWaitSeconds} under {@code Autonomous} shuffleboard tab.
	 */
	public WaitCommand() {
		this("AutoWaitSeconds");
	}

	/**
	 * Creates a new DashboardWaitCommand under {@code Autonomous} shuffleboard tab.
	 * @param key Name of {@link edu.wpi.first.networktables.NetworkTableEntry Network Table Entry}
	 */
	public WaitCommand(String key) {
		this(key, "Autonomous");
	}

	/**
	 * Creates a new DashboardWaitCommand.
	 * @param key Name of {@link edu.wpi.first.networktables.NetworkTableEntry Network Table Entry}
	 * @param tab_name Name of shuffleboard tab for the entry
	 */
	public WaitCommand(String key, String tab_name) {
		final var tab = Shuffleboard.getTab(tab_name);

		for (final var elem : tab.getComponents()) {
			if (elem.getTitle().equals(key) && elem instanceof SimpleWidget) {
				entry = ((SimpleWidget) elem).getEntry();
			}
		}

		if (entry == null) {
			entry = Shuffleboard.getTab(tab_name).add(key, 0d).withWidget(BuiltInWidgets.kNumberSlider)
					.withProperties(Map.of("min", 0, "max", 15)).getEntry();
		}
	}

	@Override
	public void initialize() {
		final double time = (entry != null) ? entry.getDouble(0) : 0;
		System.out.println(time + "   <-----------------------TIME!!!!");
		final double startTime = Timer.getFPGATimestamp();
		targetTime = startTime + time;
	}

	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() >= targetTime;
	}

}
