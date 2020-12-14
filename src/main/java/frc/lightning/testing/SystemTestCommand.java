/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lightning.testing;

import java.util.Iterator;
import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lightning.fault.FaultCode;

public class SystemTestCommand extends CommandBase {

	private static PriorityQueue<SystemTest> queue = new PriorityQueue<>();

	private Iterator<SystemTest> itor;
	
    private SystemTest current;

	private static final ShuffleboardTab tab = Shuffleboard.getTab("System Tests");

	private boolean passedAll = true;

	private boolean isResting = false;
    
    private double restStarted = 0d;

	private final double restDuration = 2d;
	
	private UserIndication userIndication = UserIndication.NONE;

	private enum UserIndication {
		NONE, SUCCESS, FAILURE
	}

	private void resetUserIndication() {
		userIndication = UserIndication.NONE;
	}

	public static void register(SystemTest test) {
		queue.add(test);
		tab.getLayout("Tests", BuiltInLayouts.kList).add(test.getName(), queue.size());
	}

	public SystemTestCommand() {
		System.out.println("SystemTest.constructor");

		tab.addNumber("Number Tests Remaining", () -> queue.size());

		tab.addString("Current Test", () -> {
			if(current != null) return current.getMessage();
			return "";
		});

		var functionList = tab.getLayout("Test Functions", BuiltInLayouts.kList);

		functionList.add("Indicate Success", new InstantCommand(() -> {
			userIndication = UserIndication.SUCCESS;
			current.cancel();
		}));

		functionList.add("Indicate Failure", new InstantCommand(() -> {
			userIndication = UserIndication.FAILURE;
			current.cancel();
		}));

		for (var test : queue) {
            for (var req : test.getRequirements()) {
                addRequirements(req);
            }
		}
		
	}

	@Override
	public void initialize() {
		System.out.println("SystemTest.initialize");
		itor = queue.iterator();
	}

	@Override
	public void execute() {

		if(isResting) {
            if(Timer.getFPGATimestamp() - restStarted < restDuration) return;
            isResting = false;
        }

        if (current == null) {
			System.out.println("SystemTest.nextTest");
            current = itor.next();
            current.initialize();
		}
		
        if (current.isFinished()) {

			current.end(false);
			
			if(userIndication == UserIndication.SUCCESS) {
				System.out.println("SystemTest.passedByUserIndication");
				onPass();
			} else if(userIndication == UserIndication.FAILURE) {
				System.out.println("SystemTest.failedByUserIndication");
				onFail();
			} else { // no user indication
				if(current.didPass()) onPass();
				else onFail();
			}
			
            current = null;
            isResting = true;
			restStarted = Timer.getFPGATimestamp();

			resetUserIndication();
			
        } else {
            current.execute();
		}
		
	}

	private void onPass() {
		System.out.println("SystemTest.passed:" + current.getName());
	}

	private void onFail() {
		FaultCode.write(current.getCode());
		System.out.println("SystemTest.failed:" + current.getName());
		passedAll = false;
	}

	@Override
	public void end(boolean interrupted) {
		if (current != null) {
            current.end(interrupted);
        }
        System.out.println(passedAll ? "SystemTest.passedAll" : "SystemTest.didNotPassAll");
	}

	@Override
	public boolean isFinished() {
		boolean condition = ((current == null) && (!itor.hasNext()));
		if(condition) System.out.println("");
		return condition;
	}

}
