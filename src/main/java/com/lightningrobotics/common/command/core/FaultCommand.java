package com.lightningrobotics.common.command.core;

import com.lightningrobotics.common.fault.FaultCode;
import com.lightningrobotics.common.fault.LightningFaultCodes;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Command to trigger a {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code}
 */
public class FaultCommand extends InstantCommand {
    final LightningFaultCodes.Code code;

    /**
     * Instantly writes the given {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code}
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to write
     */
    public FaultCommand(LightningFaultCodes.Code code) {
        this.code = code;
    }

    @Override
    public void initialize() {
        System.out.println("FaultCommand initialize: " + code);
        FaultCode.write(code);
    }

}
