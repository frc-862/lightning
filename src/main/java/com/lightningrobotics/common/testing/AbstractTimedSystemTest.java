package com.lightningrobotics.common.testing;

import edu.wpi.first.wpilibj.Timer;
import com.lightningrobotics.common.fault.LightningFaultCodes;

/**
 * Abstract {@link SystemTest} that runs for a given period of time.
 */
public abstract class AbstractTimedSystemTest extends SystemTest {

    private double timeAtStart;
    
    private final double timeout;

    /**
     * Creates a new AbstractTimedSystemTest
     * @param msg Dashboard display message for the SystemTest
     * @param timeout Duration the test should be run in seconds
     * @param code {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code FaultCode} relating
     * to the SystemTest
     */
    public AbstractTimedSystemTest(String msg, double timeout, LightningFaultCodes.Code code) {
        this(msg, timeout, code, Priority.DONT_CARE);
    }

    /**
     * Creates a new AbstractTimedSystemTest
     * @param msg Dashboard display message for the SystemTest
     * @param timeout Duration the test should be run in seconds
     * @param code {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code FaultCode} relating
     * to the SystemTest
     * @param priority {@link SystemTest.Priority} of the SystemTest
     */
    public AbstractTimedSystemTest(String msg, double timeout, LightningFaultCodes.Code code, Priority priority) {
        super(msg, code, priority);
        this.timeout = timeout;
    }

    @Override
    public void initialize() {
        timeAtStart = Timer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return elapsed() > timeout;
    }

    /**
     * The quantity of time that has passed since the test
     * has started running.
     * @return The elapsed time in seconds.
     */
    protected double elapsed() {
        return Timer.getFPGATimestamp() - timeAtStart;
    }
}