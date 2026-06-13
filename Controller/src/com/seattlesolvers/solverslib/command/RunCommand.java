package com.seattlesolvers.solverslib.command;
public class RunCommand {
    public RunCommand(Runnable action) {}
    public RunCommand(Runnable action, SubsystemBase... requirements) {}
}
