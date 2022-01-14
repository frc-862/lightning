package com.lightningrobotics.common.command.core;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

public class LogCommand extends CommandBase {
    final Supplier<String> msg;

    public LogCommand(Supplier<String> msg) {
        this.msg = msg;
    }

    public LogCommand(String msg) {
        this.msg = () -> msg;
    }

    @Override
    public void initialize() {
        System.out.println(msg.get());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() { }

    @Override
    public void end(boolean interrupted) { }
}

