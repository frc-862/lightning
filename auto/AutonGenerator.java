package frc.lightning.auto;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public class AutonGenerator {

    private HashMap<String, Command> map = new HashMap<>();

    public HashMap<String, Command> getCommands() { return map; }
    
    public void registerCommand(String name, Command cmd) { map.put(name, cmd); } 
    
}
