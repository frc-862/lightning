package com.lightningrobotics.common;

import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link LightningRobot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared in a subclass of this class.
 */
public abstract class LightningContainer {

    public LightningContainer() {

        // Setup driver commands and defaults
        configureButtonBindings();
        configureDefaultCommands();
        initializeDashboardCommands();

        // Configure autonomous
        configureAutonomousCommands();

        // Setup fault monitoring
        configureFaultCodes();
        configureFaultMonitors();

        // Setup system tests
        configureSystemTests();

    }

    /**
     * Connects commands with buttons on joysticks
     */
    protected abstract void configureButtonBindings();

    /**
     * Registers all systems tests for the robot to be run
     */
    protected abstract void configureSystemTests();

    /**
     * Configures all default commands to run on subsystems when no other command
     * requires that subsystem
     */
    protected abstract void configureDefaultCommands();

    /**
     * Cancles all default commands
     */
    protected abstract void releaseDefaultCommands();

    /**
     * Puts command buttons on the dashboard
     */
    protected abstract void initializeDashboardCommands();

    /**
     * Configures list of possible commands that can be run during autonomous
     */
    protected abstract void configureAutonomousCommands();

    /**
     * Configures all robot-specific fault codes
     */
    protected abstract void configureFaultCodes();

    /**
     * Configures listeners for fault codes
     */
    protected abstract void configureFaultMonitors();

    /**
     * The {@link com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain drivetrain} object of the robot
     * @return The drivetrain object of the given robot
     */
    public abstract LightningDrivetrain getDrivetrain();

}
