package com.lightningrobotics.common.command.drivetrain.differential;

import java.util.function.DoubleSupplier;

import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.util.filter.JoystickFilter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DifferentialTankDrive extends CommandBase {

    DifferentialDrivetrain drivetrain;
    DoubleSupplier left;
    DoubleSupplier right;
    private double deadband = 0.15;
    private double minPower = 0.1;
    private double maxPower = 1.0;
    private final JoystickFilter filter = new JoystickFilter(deadband, minPower, maxPower, JoystickFilter.Mode.CUBED);

    public DifferentialTankDrive(DifferentialDrivetrain drivetrain, DoubleSupplier left, DoubleSupplier right) {
        this.drivetrain = drivetrain;
        this.left = left;
        this.right = right;
        addRequirements(drivetrain);
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