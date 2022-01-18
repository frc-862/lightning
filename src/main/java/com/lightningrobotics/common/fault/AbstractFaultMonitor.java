package com.lightningrobotics.common.fault;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.lightningrobotics.common.fault.LightningFaultCodes.Code;

/**
 * Base {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} checking functionality
 */
public abstract class AbstractFaultMonitor {
    protected Code code;
    protected String msg;
    protected boolean fatal;

    /**
     * Creates a new abstract fault monitor
     * @param code  The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to be monitored
     * @param msg   The message to use should the fault be written
     * @param fatal If the fault is fatal and should stop the robot
     */
    public AbstractFaultMonitor(Code code, String msg, boolean fatal) {
        this.code = code;
        this.msg = msg;
        this.fatal = fatal;
    }

    /**
     * Creates a new non-fatal abstract fault monitor
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to be monitored
     * @param msg  The message to use should the fault be written
     */
    public AbstractFaultMonitor(Code code, String msg) {
        this(code, msg, false);
    }

    /**
     * Creates a new non-fatal abstract fault monitor
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to be monitored
     */
    public AbstractFaultMonitor(Code code) {
        this(code, "FAULT " + code.getName(), false);
    }

    /**
     * To be overridden, determines if fault should be written
     * @return True if the fault should be written, false otherwise
     */
    public abstract boolean checkFault();

    /**
     * Checks if the fault should be written, and writes the fault if it should
     * @return True if the fault should be written, false otherwise
     */
    public boolean check() {
        if (checkFault()) {
            trigger();
            return true;
        }
        return false;
    }

    /**
     * Writes the fault. If the fault is fatal, cancles and disables the
     * {@link edu.wpi.first.wpilibj2.command.CommandScheduler}
     */
    public void trigger() {
        FaultCode.write(code, msg);
        if (fatal) {
            CommandScheduler.getInstance().cancelAll();
            CommandScheduler.getInstance().disable();
        }
    }
    
}
