package com.lightningrobotics.quasar;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot Application Driver Class.
 * 
 * <p>Starts Robot Application.
 */
public final class Main {
    private Main() {
    }

    /**
     * Main initialization function. 
     * 
     * @param args Command line arguments
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
