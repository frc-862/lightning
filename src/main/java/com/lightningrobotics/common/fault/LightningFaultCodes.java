package com.lightningrobotics.common.fault;

import java.util.ArrayList;
import java.util.List;

public class LightningFaultCodes {

    public static class Code {

        private String name;

        private boolean state;

        public Code(String name, boolean state) {
            this.name = name;
            this.state = state;
        }

        public Code(String name) {
            this(name, false);
        }

        public String getName() {
            return name;
        }

        public void setName(String name) {
            this.name = name;
        }

        public boolean getState() {
            return state;
        }

        public void setState(boolean state) {
            this.state = state;
        }

    }

    private static List<Code> codes = new ArrayList<>();

    static {
        codes.add(new Code("GENERAL_FAULT"));
        codes.add(new Code("LOW_MAIN_VOLTAGE"));
        codes.add(new Code("SLOW_LOOPER"));
        codes.add(new Code("MISMATCHED_MOTION_PROFILES"));
        codes.add(new Code("NAVX_ERROR"));
        codes.add(new Code("INTERNAL_ERROR"));
        codes.add(new Code("DRIVETRAIN"));
    }

    public static void addFaultCode(String name, boolean state) {
        addFaultCode(new Code(name, state));        
    }

    public static void addFaultCode(String name) {
        addFaultCode(new Code(name, false));
    }

    public static void addFaultCode(Code code) {
        codes.add(code);
    }

    public static Code getFaultCode(String name) {
        for (var code : codes) {
            if (code.getName().equals(name)) {
                return code;
            }
        }
        return getFaultCode("GENERAL_FAULT");
    }
    
}
