package com.lightningrobotics.common.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.lightningrobotics.common.fault.FaultCode;

/**
 * Base testing class. System tests run when the robot is enabled in test mode 
 * on the Driver Station. System tests are run with the 
 * {@link com.lightningrobotics.common.testing.SystemTestCommand SystemTestCommand} which is configured to run
 * automatically in test mode. Individual system tests can be queued by calling 
 * {@link SystemTest#register(SystemTest)}.
 */
public abstract class SystemTest extends CommandBase implements Comparable<SystemTest> {

    private String msg;

    /**
     * Registers the given system test to be run when the 
     * {@link com.lightningrobotics.common.LightningRobot LightningRobot} is
     * enabled in test mode.
     * @param test The system test to be registered.
     */
    public static void register(SystemTest test) {
        SystemTestCommand.register(test);
    }

    /**
     * Importance of the SystemTest. SystemTests with higher
     * priorities are run first.
     */
    public enum Priority {
        DO_FIRST, HIGH, MED, LOW, DONT_CARE
    }

    private FaultCode.Codes code;
    private Priority priority;

    /**
     * Creates a new SystemTest
     * @param msg Dashboard display message for the SystemTest
     * @param code {@link com.lightningrobotics.common.fault.FaultCode.Codes FaultCode} relating
     * to the SystemTest
     */
    public SystemTest(String msg, FaultCode.Codes code) {
        this(msg, code, Priority.DONT_CARE);
    }

    /**
     * Creates a new SystemTest
     * @param msg Dashboard display message for the SystemTest
     * @param code {@link com.lightningrobotics.common.fault.FaultCode.Codes FaultCode} relating
     * to the SystemTest
     * @param priority {@link SystemTest.Priority} of the SystemTest
     */
    public SystemTest(String msg, FaultCode.Codes code, Priority priority) {
        this.msg = msg;
        this.code = code;
        this.priority = priority;
    }

    /**
     * The priority of the SystemTest
     * @return The {@link SystemTest.Priority} of the SystemTest
     */
    public Priority getPriority() {
        return priority;
    }

    /**
     * The dashboard message of the SystemTest
     * @return The message to be displayed on the dashboard
     */
    public String getMessage() {
        return msg;
    }

    /**
     * Determines passing conditions for the SystemTest
     * @return true if the test was completed successfully, false otherwise
     */
    abstract public boolean didPass();

    /**
     * The relating {@link com.lightningrobotics.common.fault.FaultCode.Codes FaultCode} 
     * corresponding to the SystemTest
     * @return The corresponding FaultCode
     */
    public FaultCode.Codes getCode() {
        return code;
    }

    @Override
    public int compareTo(SystemTest other) {
        return priority.compareTo(other.priority);
    }

}