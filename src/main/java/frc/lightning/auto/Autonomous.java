package frc.lightning.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lightning.commands.DashboardWaitCommand;

public class Autonomous {

    private static int autonCommandCount = 0; 

    private static boolean wait = true;

    private static HashMap<String, Command> autons = new HashMap<>();

    private static SendableChooser<Command> chooser = new SendableChooser<>();

    public static HashMap<String, Command> getCommands() { return autons; }
    
    public static void register(String name, Command cmd) { autons.put(name, cmd); } 

    public static boolean hasDashboardWaitCommand() { return wait; }

    public static void setHasDashboardWaitCommand(boolean hasWait) {
        wait = hasWait;
    }

    public static void load() {
        final var tab = Shuffleboard.getTab("Autonomous");
        Set<String> names = autons.keySet();
        for(var name : names) {
            loadRegisteredCommand(name, autons.get(name));
            System.out.println("Registered " + name + " command for auton");
        }
        tab.add("Auto Mode", chooser);
    }

    public Command getCommand() {
        if(wait) {
            return new SequentialCommandGroup(new DashboardWaitCommand(), chooser.getSelected());
        } else {
            return chooser.getSelected();
        }
    }

    private static void loadRegisteredCommand(String name, Command command) {
        if (autonCommandCount == 0) {
            chooser.setDefaultOption(name, command);
        } else {
            chooser.addOption(name, command);
        }
        autonCommandCount++;
    }

    private static void loadRegisteredCommand(Command command) {
        loadRegisteredCommand(command.getName(), command);
    }
    
}
