package frc.lightning.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lightning.LightningConfig;
import frc.lightning.subsystems.LightningDrivetrain;
import frc.lightning.util.JoystickFilter;

public class VoltDrive extends CommandBase {

    private static final double DEFAULT_DEADBAND = 0.15;
    private static final double DEFAULT_MIN_PWR = 0.1;
    private static final double DEFAULT_MAX_PWR = 1.0;

    
    private final JoystickFilter filter;

    LightningDrivetrain drivetrain;
    DoubleSupplier left;
    DoubleSupplier right;

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this(drivetrain, left, right, JoystickFilter.Mode.CUBED);
    }

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right, JoystickFilter.Mode mode) {
        this(drivetrain, left, right, mode, DEFAULT_DEADBAND, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR);
    }

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right, JoystickFilter.Mode mode, double deadband) {
        this(drivetrain, left, right, mode, deadband, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR);
    }

    public VoltDrive(LightningDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right, JoystickFilter.Mode mode, double deadband, double minPower, double maxPower) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;
        addRequirements(drivetrain);
        filter = new JoystickFilter(deadband, minPower, maxPower, mode);
    }

    @Override
    public void execute() {

        double leftVolts = filter.filter(left.getAsDouble());
        double rightVolts = filter.filter(right.getAsDouble());

        leftVolts  *= LightningConfig.VOLT_LIMIT; // RobotController.getBatteryVoltage(); ?
        rightVolts *= LightningConfig.VOLT_LIMIT;

        drivetrain.setOutput(leftVolts, rightVolts);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}