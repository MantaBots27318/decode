package com.seattlesolvers.solverslib.command;
public class InstantCommand {
    public InstantCommand(Runnable action) {}
    public InstantCommand(Runnable action, SubsystemBase... requirements) {}
}
