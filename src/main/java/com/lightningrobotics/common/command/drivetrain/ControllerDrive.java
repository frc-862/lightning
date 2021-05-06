package com.lightningrobotics.common.command.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.differential.DifferentialDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.swerve.SwerveDrivetrain;
import com.lightningrobotics.common.util.operator.JoystickFilter;
import com.lightningrobotics.common.util.operator.JoystickFilter.Mode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControllerDrive extends CommandBase {

    public enum ControlType {
        TANK, ARCADE, SWERVE_ROBOT_RELATIVE, SWERVE_FIELD_RELATIVE
    }

    private static final double DEFAULT_DEADBAND = 0.05;
    private static final double DEFAULT_MIN_PWR = 0.1;
    private static final double DEFAULT_MAX_PWR = 1.0;
    private static final JoystickFilter DEFAULT_FILTER = new JoystickFilter(DEFAULT_DEADBAND, DEFAULT_MIN_PWR, DEFAULT_MAX_PWR, Mode.SQUARED);
    
    private final JoystickFilter filter;

    private final ControlType controlType;

    private LightningDrivetrain drivetrain;
    private DifferentialDrivetrain differentialDrivetrain;
    private SwerveDrivetrain swerveDrivetrain;

    List<DoubleSupplier> inputs = new ArrayList<>();

    public static ControllerDrive tank(LightningDrivetrain drivetrain, JoystickFilter filter, DoubleSupplier leftInput, DoubleSupplier rightInput) {
        // inputs = left, right
        return new ControllerDrive(ControlType.TANK, drivetrain, filter, leftInput, rightInput);
    }

    public static ControllerDrive tank(LightningDrivetrain drivetrain, DoubleSupplier leftInput, DoubleSupplier rightInput) {
        // inputs = left, right
        return tank(drivetrain, DEFAULT_FILTER, leftInput, rightInput);
    }

    public static ControllerDrive arcade(LightningDrivetrain drivetrain, JoystickFilter filter, DoubleSupplier powerInput, DoubleSupplier rotationInput) {
        // inputs = speed, rotation
        return new ControllerDrive(ControlType.ARCADE, drivetrain, filter, powerInput, rotationInput);
    }

    public static ControllerDrive arcade(LightningDrivetrain drivetrain, DoubleSupplier powerInput, DoubleSupplier rotationInput) {
        // inputs = speed, rotation
        return arcade(drivetrain, DEFAULT_FILTER, powerInput, rotationInput);
    }

    public static ControllerDrive swerveRobotRelative(LightningDrivetrain drivetrain, DoubleSupplier... inputs) { // TODO implement
        return new ControllerDrive(ControlType.SWERVE_ROBOT_RELATIVE, drivetrain, inputs);
    }

    public static ControllerDrive swerveFieldRelative(LightningDrivetrain drivetrain, DoubleSupplier... inputs) { // TODO implement
        return new ControllerDrive(ControlType.SWERVE_FIELD_RELATIVE, drivetrain, inputs);
    }

    private ControllerDrive(ControlType controlType, LightningDrivetrain drivetrain, DoubleSupplier... inputs) {
        this(controlType, drivetrain, DEFAULT_FILTER, inputs);
    }

    private ControllerDrive(ControlType controlType, LightningDrivetrain drivetrain, JoystickFilter filter, DoubleSupplier... inputs) {
        this.controlType = controlType;
        this.drivetrain = drivetrain;
        this.filter = filter;

        addRequirements(drivetrain);
        for(DoubleSupplier input : inputs) this.inputs.add(input);
    }

    @Override
    public void initialize() {
        super.initialize();
        if((controlType == ControlType.TANK || controlType == ControlType.ARCADE) && drivetrain instanceof DifferentialDrivetrain) {
            differentialDrivetrain = (DifferentialDrivetrain) drivetrain;
        } else if((controlType == ControlType.SWERVE_ROBOT_RELATIVE || controlType == ControlType.SWERVE_FIELD_RELATIVE) && drivetrain instanceof SwerveDrivetrain) {
            swerveDrivetrain = (SwerveDrivetrain) drivetrain;
        }
    }

    @Override
    public void execute() {

        switch (controlType) {
            case TANK:
                var leftSpeed = filter.filter(inputs.get(0).getAsDouble());
                var rightSpeed = filter.filter(inputs.get(1).getAsDouble());
                differentialDrivetrain.tankDrive(leftSpeed, rightSpeed);
                break;
            case ARCADE:
                var speed = filter.filter(inputs.get(0).getAsDouble());
                var rot = filter.filter(inputs.get(1).getAsDouble());
                differentialDrivetrain.arcadeDrive(speed, rot);
                break;
            case SWERVE_ROBOT_RELATIVE:
                swerveDrivetrain.stop();
                break;
            case SWERVE_FIELD_RELATIVE:
                break;
            default:
                drivetrain.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stop();
    }

}
