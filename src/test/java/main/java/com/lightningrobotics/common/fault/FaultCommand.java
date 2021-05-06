package com.lightningrobotics.common.fault;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Command to trigger a {@link com.lightningrobotics.common.fault.FaultCode.Codes}
 */
public class FaultCommand extends InstantCommand {
    final FaultCode.Codes code;

    /**
     * Instantly writes the given {@link com.lightningrobotics.common.fault.FaultCode.Codes}
     * @param code The {@link com.lightningrobotics.common.fault.FaultCode.Codes} to write
     */
    public FaultCommand(FaultCode.Codes code) {
        this.code = code;
    }

    @Override
    public void initialize() {
        System.out.println("FaultCommand initialize: " + code);
        FaultCode.write(code);
    }

}
