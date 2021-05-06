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

        public String getName() {
            return name;
        }

        public void setName(String name) {
            this.name = name;
        }

        public boolean isState() {
            return state;
        }

        public void setState(boolean state) {
            this.state = state;
        }

    }

    private static List<Code> codes = new ArrayList<>();

    public static void addFaultCode(String name, boolean state) {
        addFaultCode(new Code(name, state));        
    }

    public static void addFaultCode(String name) {
        addFaultCode(new Code(name, false));
    }

    public static void addFaultCode(Code code) {
        codes.add(code);
    }

    public static Code getFaultCode() {
        // TODO finish implementing
        return null;
    }
    
}
