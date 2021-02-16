package frc.lightning.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lightning.LightningConfig;
import frc.lightning.subsystems.LightningDrivetrain;
import frc.lightning.util.JoystickFilter;

/**
 * Simple voltage-based open loop drive. 
 * Uses {@link frc.lightning.subsystems.LightningDrivetrain LightningDrivetrain}
 */
public class VoltDrive extends CommandBase {

    LightningDrivetrain drivetrain;
    DoubleSupplier left;
    DoubleSupplier right;
    private final double DEADBAND = 0.15;
    private final double MIN_ALLOWED_PWR = 0.1;
    private final double MAX_ALLOWED_PWR = 1.0;
    private final JoystickFilter filter;

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this(drivetrain, left, right, JoystickFilter.Mode.CUBED);
    }

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right, JoystickFilter.Mode jsRamp) {
        filter = new JoystickFilter(DEADBAND, MIN_ALLOWED_PWR, MAX_ALLOWED_PWR, jsRamp);
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double leftVolts = filter.filter(left.getAsDouble());
        double rightVolts = filter.filter(right.getAsDouble());

        leftVolts  *= LightningConfig.VOLT_LIMIT;
        rightVolts *= LightningConfig.VOLT_LIMIT;

        drivetrain.setOutput(leftVolts, rightVolts);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}