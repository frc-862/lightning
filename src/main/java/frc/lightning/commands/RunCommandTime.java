/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lightning.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunCommandTime extends CommandBase {

	private Command cmd;
	private double runTime, startTime;

	/**
	 * Creates a new RunCommandTime with the specified runtime
	 * @param cmd The command to be run
	 * @param runTime The duration the command should be run
	 */
	public RunCommandTime(Command cmd, double runTime) {
		this.cmd = cmd;
		this.runTime = runTime;
		for (var sys : cmd.getRequirements()) addRequirements(sys);
	}

	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
		cmd.initialize();
	}

	@Override
	public void execute() {
		cmd.execute();
	}

	@Override
	public void end(boolean interrupted) {
		cmd.end(interrupted);
	}

	@Override
	public boolean isFinished() {
		return (((Timer.getFPGATimestamp() - startTime) >= runTime) || cmd.isFinished());
	}
}
