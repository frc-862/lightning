package frc.lightning.fault;

import frc.lightning.fault.FaultCode.Codes;

public abstract class AbstractFaultMonitor {
    protected Codes code;
    protected String msg;

    public AbstractFaultMonitor(Codes code, String msg) {
        this.code = code;
        this.msg = msg;
    }

    public AbstractFaultMonitor(Codes code) {
        this(code, "FAULT " + code.name());
    }

    public abstract boolean checkFault();

    public boolean check() {
        if (checkFault()) {
            trigger();
            return true;
        }
        return false;
    }

    public void trigger() {
        FaultCode.write(code, msg);
    }
}