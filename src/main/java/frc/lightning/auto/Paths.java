package frc.lightning.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lightning.subsystems.LightningDrivetrain;

public class Paths {

    private Map<String, Path> paths = new HashMap<>();

    public Map<String, Path> getPaths() { return paths; }

    public Map<String, Command> getPathCommands(LightningDrivetrain drivetrain) {
        Map<String, Command> cmds = new HashMap<>();
        List<String> keys = new ArrayList<>(getPaths().keySet());
        for(int i = 0 ; i < getPaths().size() ; ++i) {
            String name = keys.get(i);
            Command cmd = getPaths().get(name).getCommand(drivetrain);
            cmds.put(name, cmd);
        }
        return cmds;
    }

}