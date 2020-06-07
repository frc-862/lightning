/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lightning;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lightning.auto.Path;
import frc.lightning.subsystems.LightningDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared in a subclass of this class.
 */
public abstract class LightningContainer {

    protected abstract void configureButtonBindings();

    protected abstract void configureSystemTests();

    protected abstract void configureDefaultCommands();

    protected abstract void releaseDefaultCommands();

    protected abstract void initializeDashboardCommands();

    public abstract List<Path> getAutonomousPaths();

    public abstract LightningDrivetrain getDrivetrain();

    public abstract HashMap<String, Command> getAutonomousCommands();

}
