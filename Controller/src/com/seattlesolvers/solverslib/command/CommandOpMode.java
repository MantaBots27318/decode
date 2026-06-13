package com.seattlesolvers.solverslib.command;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
public abstract class CommandOpMode extends OpMode {
    public abstract void initialize();
    @Override public void init() { reset(); initialize(); }
    @Override public void loop() { run(); }
    public void run() {}
    public void reset() {}
    public void schedule(Object... commands) {}
}
