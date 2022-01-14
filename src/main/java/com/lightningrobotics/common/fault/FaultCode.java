package com.lightningrobotics.common.fault;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * Error codes for robot and means to log them
 */
public class FaultCode {

    private static final String FAULT_PATH = "/home/lvuser/faults.log";
    private static HashSet<LightningFaultCodes.Code> faults = new HashSet<>();
    private static Map<LightningFaultCodes.Code, NetworkTableEntry> networkTableMap = new HashMap<>();
    private static boolean dummy_light = false;

    /**
     * Links a {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} with a {@link edu.wpi.first.networktables.NetworkTableEntry}
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} that needs to be linked with an NT entry
     * @param nte The {@link edu.wpi.first.networktables.NetworkTableEntry} to link the fault to
     */
    public static void setNetworkTableEntry(LightningFaultCodes.Code code, NetworkTableEntry nte) {
        networkTableMap.put(code, nte);
    }

    /**
     * Push each fault code to the dashboard
     */
    public static void init() {
        eachCode((LightningFaultCodes.Code code, Boolean state) -> {
            var nte = Shuffleboard.getTab("Fault Codes")
                    .add("FAULT_" + code.toString(), state)
                    .withWidget("Boolean Box")
                    .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
                    .getEntry();
            FaultCode.setNetworkTableEntry(code, nte);
        });
        try {
            Files.write(getFaultPath(), ("######### RESTART #########\n").getBytes(), StandardOpenOption.CREATE,
                        StandardOpenOption.APPEND);
        } catch (IOException e) {
            System.err.println("Unable to append to fault log file: " + getFaultPath() + ": " + e);
            e.printStackTrace();
        }
    }

    /**
     * Fault path log file
     * @return The path to the fault log file
     */
    private static Path getFaultPath() {
        return Paths.get(FAULT_PATH);
    }

    /**
     * Writes the given fault code
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to be written
     */
    public static void write(LightningFaultCodes.Code code) {
        write(code, "");
    }

    /**
     * Updates the {@link edu.wpi.first.networktables.NetworkTableEntry} for each 
     * {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code}.
     */
    public static void update() {
        eachCode((LightningFaultCodes.Code c, Boolean state) -> {
            final var entry = networkTableMap.get(c);
            if (entry != null) entry.setBoolean(state);
        });
    }

    /**
     * Runs the given function on each code. 
     * 
     * @param fn The {@link java.util.function.BiConsumer} function to perform on each code. 
     * The function should take a code and state (true or false) as parameters.
     */
    public static void eachCode(BiConsumer<LightningFaultCodes.Code, Boolean> fn) {
        for (LightningFaultCodes.Code c : LightningFaultCodes.getCodes()) {
            fn.accept(c, !faults.contains(c));
        }
    }

    /**
     * Writes the fault code, effectively logging that it has been detected in 
     * a log file and the system error stream. Per {@link #update()}, they will be updated
     * automatically on {@link edu.wpi.first.wpilibj.shuffleboard.Shuffleboard Shuffleboard}
     * @param code The {@link com.lightningrobotics.common.fault.LightningFaultCodes.Code} to be written
     * @param msg The message to write to the log file
     */
    public static void write(LightningFaultCodes.Code code, String msg) {
        dummy_light = true;
        try {
            if (!faults.contains(code)) {
                faults.add(code);
                code.setState(true);
                Files.write(getFaultPath(),
                            ("FAULT Detected: " + code.toString() + " " + msg + "\n").getBytes(),
                            StandardOpenOption.CREATE,
                            StandardOpenOption.APPEND);
                System.err.println("FAULT: " + code + " " + msg);
            }
        } catch (IOException e) {
            System.err.println("Unable to write fault code " + code);
            e.printStackTrace();
        }
    }
    public boolean dummyLightOn() {
        return dummy_light;
    }

    /**
     * Gets all the faults as a JSON String
     * @return The faults in JSON format
     */
    public static String toJSONString() {
        return faults.stream().map((c) -> "\"" + c.toString() + "\"").collect(Collectors.joining(",","[","]"));
    }

    /**
     * Gets a model of the faults and their states as well as the timestamp
     * @return The model of faults
     */
    public static Map<String, Object> getModel() {
        Map<String, Object> result = new HashMap<>();
        Map<String, Object> faults = new HashMap<>();

        eachCode((LightningFaultCodes.Code c, Boolean state) -> {
            faults.put("FAULT_" + c.toString(), state);
        });

        result.put("faults", faults);
        result.put("timer", Timer.getFPGATimestamp());

        return result;
    }

}
