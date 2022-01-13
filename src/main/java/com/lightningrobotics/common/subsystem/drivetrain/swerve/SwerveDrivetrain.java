package com.lightningrobotics.common.subsystem.drivetrain.swerve;

import java.util.function.Consumer;

import com.lightningrobotics.common.geometry.kinematics.DrivetrainSpeed;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveDrivetrainState;
import com.lightningrobotics.common.geometry.kinematics.swerve.SwerveModuleState;
import com.lightningrobotics.common.subsystem.drivetrain.LightningDrivetrain;
import com.lightningrobotics.common.subsystem.drivetrain.LightningGains;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public abstract class SwerveDrivetrain extends LightningDrivetrain {

    public enum Modules {
        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        REAR_LEFT(2),
        REAR_RIGHT(3);
        private int moduleID;
        Modules(int moduleID) { this.moduleID = moduleID; }
        public int getModuleID() { return moduleID; }
    }

    private SwerveGains gains;

    private SwerveModule[] modules;

    private SwerveDrivetrainState state;
    private DrivetrainSpeed speed;

    public SwerveDrivetrain(SwerveGains gains, SwerveModule... modules) {
        this.gains = gains;
        this.modules = modules;

        this.state = new SwerveDrivetrainState(new SwerveModuleState[]{
            new SwerveModuleState(0d, modules[Modules.FRONT_LEFT.getModuleID()].getModuleAngle()),
            new SwerveModuleState(0d, modules[Modules.FRONT_RIGHT.getModuleID()].getModuleAngle()),
            new SwerveModuleState(0d, modules[Modules.REAR_LEFT.getModuleID()].getModuleAngle()),
            new SwerveModuleState(0d, modules[Modules.REAR_RIGHT.getModuleID()].getModuleAngle())
        });

        // Initialize zero drive speed
        speed = new DrivetrainSpeed();

        // Put some data on shuffleboard
        var tab = Shuffleboard.getTab("Swerve Module States");

        tab.addString("FL Real", () -> modules[Modules.FRONT_LEFT.getModuleID()].getState().toString());
        tab.addString("FL Target", () -> state.getStates()[Modules.FRONT_LEFT.getModuleID()].toString());

        tab.addString("FR Real", () -> modules[Modules.FRONT_RIGHT.getModuleID()].getState().toString());
        tab.addString("FR Target", () -> state.getStates()[Modules.FRONT_RIGHT.getModuleID()].toString());

        tab.addString("RL Real", () -> modules[Modules.REAR_LEFT.getModuleID()].getState().toString());
        tab.addString("RL Target", () -> state.getStates()[Modules.REAR_LEFT.getModuleID()].toString());

        tab.addString("RR Real", () -> modules[Modules.REAR_RIGHT.getModuleID()].getState().toString());
        tab.addString("RR Target", () -> state.getStates()[Modules.REAR_RIGHT.getModuleID()].toString());

        tab.addString("Target Speed", () -> speed.toString());
        tab.addString("Real Speed", () -> gains.getKinematics().forward(state).toString());

    }

    @Override
    public void configureMotors() {
        var driveInverts = gains.getDriveMotorInverts();
        var turnInverts = gains.getTurnMotorInverts();

        if (driveInverts.length == modules.length && turnInverts.length == modules.length) {
            for (var i = 0; i < modules.length; ++i) {
                modules[i].getDriveMotor().setInverted(driveInverts[i]);
                modules[i].getAzimuthMotor().setInverted(turnInverts[i]);
            }
        }
    }

    @Override
    public void setDriveSpeed(DrivetrainSpeed speed) {
        //var states = new SwerveDrivetrainSpeed(speed, gains).getStates();
        var state = (SwerveDrivetrainState) gains.getKinematics().inverse(speed);
        var swerveModuleStates = state.getStates();

        for (int i = 0; i < swerveModuleStates.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState moduleState = swerveModuleStates[i];
            module.setState(moduleState);
        }
    }

    @Override
    public LightningGains getGains() {
        return gains;
    }

    @Override
    public void stop() {
        this.setDriveSpeed(new DrivetrainSpeed());
    }

    protected void withEachModule(Consumer<SwerveModule> op) {
        for(var module : modules) op.accept(module);
    }

    protected void withEachDriveMotor(Consumer<MotorController> op) {
        for(var module : modules) op.accept(module.getDriveMotor());
    }

    protected void withEachAzimuthMotor(Consumer<MotorController> op) {
        for(var module : modules) op.accept(module.getAzimuthMotor());
    }

}
