package frc.lightning.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lightning.subsystems.LightningDrivetrain;

public abstract class PathGenerator {

    public abstract List<Path> getPaths();

    public List<Command> getPathCommands(LightningDrivetrain drivetrain) {
        List<Command> cmds = new ArrayList<>();
        for(var path : getPaths()) cmds.add(path.getCommand(drivetrain));
        return cmds;
    }

}