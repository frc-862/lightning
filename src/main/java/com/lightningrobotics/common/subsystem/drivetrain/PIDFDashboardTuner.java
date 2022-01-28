// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lightningrobotics.common.subsystem.drivetrain;

import com.lightningrobotics.common.controller.PIDFController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDFDashboardTuner extends SubsystemBase {

	private PIDFController controller;

	private NetworkTableEntry kPTuner;
	private NetworkTableEntry kITuner;
	private NetworkTableEntry kDTuner;

	
	public PIDFDashboardTuner(String name, PIDFController controller) {
		this.controller = controller;

		var tab = Shuffleboard.getTab(name + "_PID_Tuner"); // TODO maybe change these to just name and not + _PID_Tuner

		kPTuner = tab.add("kP", 0d).getEntry();
		kITuner = tab.add("kI", 0d).getEntry();
		kDTuner = tab.add("kD", 0d).getEntry();

		kPTuner.setDouble(controller.getP());
		kITuner.setDouble(controller.getI());
		kDTuner.setDouble(controller.getD());

	}

	public PIDFDashboardTuner(String name, PIDFController controller, String side) {
		this.controller = controller;

		var tab = Shuffleboard.getTab(name + "_PID_Tuner");

		kPTuner = tab.add("kP" + side, 0d).getEntry();
		kITuner = tab.add("kI" + side, 0d).getEntry();
		kDTuner = tab.add("kD" + side, 0d).getEntry();

		kPTuner.setDouble(controller.getP());
		kITuner.setDouble(controller.getI());
		kDTuner.setDouble(controller.getD());

	}

	@Override
	public void periodic() {

		controller.setP(kPTuner.getDouble(controller.getP()));
		controller.setI(kITuner.getDouble(controller.getI()));
		controller.setD(kDTuner.getDouble(controller.getD()));

	}

}
