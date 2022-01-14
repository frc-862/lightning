package com.lightningrobotics.common.fault;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing a set of fault codes, or enumerated things that can go wrong.
 * e.g. an encoder fault. We can use {@link com.lightningrobotics.common.fault.FaultMonitor fault monitoring} 
 * to keep tabs on these potential problems and to associate a name and error message with a problem that could happen.
 * For example, if we are applying power to a motor, but the encoder is not moving, we can write the Encoder Fault to 
 * show that something is wrong.
 */
public class LightningFaultCodes {

    /**
     * A base class that represents a single fault code
     */
    public static class Code {

        private String name;

        private boolean state;

        /**
         * Create fault code
         * @param name the name of the fault code
         * @param state the initial state of the fault code (defaults to false)
         */
        public Code(String name, boolean state) {
            this.name = name;
            this.state = state;
        }

        /**
         * Create fault code
         * @param name the name of the fault code
         */
        public Code(String name) {
            this(name, false);
        }

        /**
         * Name of the fault code
         * @return the name as a string
         */
        public String getName() {
            return name;
        }

        /**
         * Sets the name of a fault code
         * @param name the new name for the code
         */
        public void setName(String name) {
            this.name = name;
        }

        /**
         * The current state of the fault code
         * @return the boolean state of the fault code
         */
        public boolean getState() {
            return state;
        }

        /**
         * Sets the state of the fault code
         * @param state the new boolean state of the fault code
         */
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

    /**
     * Adds a fault code to the registry
     * @param name name of the fault code to be added
     * @param state initial state of the fault code
     */
    public static void addFaultCode(String name, boolean state) {
        addFaultCode(new Code(name, state));        
    }

    /**
     * Adds a fault code to the registry. The state of this code will be false
     * @param name the name of the fault code to be added
     */
    public static void addFaultCode(String name) {
        addFaultCode(new Code(name, false));
    }

    /**
     * Adds a fault code to the registry
     * @param code fault code to be added
     */
    public static void addFaultCode(Code code) {
        codes.add(code);
    }

    /**
     * Retrieve a specific fault code
     * @param name the name of the fault code
     * @return an object representing the fault code
     */
    public static Code getFaultCode(String name) {
        for (var code : codes) {
            if (code.getName().equals(name)) {
                return code;
            }
        }
        // If requested code is not in list
        return getFaultCode("GENERAL_FAULT");
    }

    /**
     * Get a list of all fault codes
     * @return a list of all of the fault codes
     */
    public static List<Code> getCodes() {
        return codes;
    }
    
}
