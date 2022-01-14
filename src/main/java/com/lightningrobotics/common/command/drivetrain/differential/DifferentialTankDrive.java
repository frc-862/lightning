package com.lightningrobotics.common.command.drivetrain.differential;

import java.util.function.DoubleSupplier;

import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.util.filter.JoystickFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DifferentialTankDrive extends CommandBase {

    private static final double DEFAULT_DEADBAND = 0.15;
    private static final double DEFAULT_MIN_PWR = 0.1;
    private static final double DEFAULT_MAX_PWR = 1.0;

    private DifferentialDrivetrain drivetrain;
    private DoubleSupplier left;
    private DoubleSupplier right;

    private JoystickFilter filter = new JoystickFilter(DEFAULT_DEADBAND, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR, JoystickFilter.Mode.CUBED);

    public DifferentialTankDrive(DifferentialDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right, JoystickFilter filter) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;
        addRequirements(drivetrain);
        this.filter = filter;
    }

    public DifferentialTankDrive(DifferentialDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this(drivetrain, left, right, new JoystickFilter(DEFAULT_DEADBAND, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR, JoystickFilter.Mode.CUBED));
    }

    @Override
    public void execute() {

        double leftPwr = filter.filter(left.getAsDouble());
        double rightPwr = filter.filter(right.getAsDouble());

        drivetrain.tankDrive(leftPwr, rightPwr);

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}