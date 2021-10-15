package com.lightningrobotics.common.fault;

import java.util.LinkedList;
import java.util.function.BooleanSupplier;

import com.lightningrobotics.common.fault.FaultCode.Codes;

import java.util.List;

/**
 * Basic implementation of an {@link com.lightningrobotics.common.fault.AbstractFaultMonitor} that 
 * also provides a means to register ad check other fault monitors
 */
public class FaultMonitor extends AbstractFaultMonitor {
    private static List<AbstractFaultMonitor> monitors = new LinkedList<>();

    final BooleanSupplier fn;

    /**
     * Adds a new fault monitor to be checked with {@link com.lightningrobotics.common.fault.FaultMonitor#checkMonitors()}
     * @param fm The {@link com.lightningrobotics.common.fault.AbstractFaultMonitor} to be registered
     */
    static public void register(AbstractFaultMonitor fm) {
        monitors.add(fm);
    }

    /**
     * Checks all fault monitors
     */
    static public void checkMonitors() {
        monitors.forEach(fm -> fm.check());
    }

    public FaultMonitor(Codes code, BooleanSupplier fn, String msg, boolean fatal) {
        super(code, msg, fatal);
        this.fn = fn;
    }

    public FaultMonitor(Codes code, BooleanSupplier fn, String msg) {
        this(code, fn, msg, false);
    }

    public FaultMonitor(Codes code, BooleanSupplier fn) {
        this(code, fn, "FAULT: " + code.toString());
    }

    @Override
    public boolean checkFault() {
        return fn.getAsBoolean();
    }

}