package com.seattlesolvers.solverslib.command;
import java.util.function.BooleanSupplier;
public class FunctionalCommand {
    public FunctionalCommand(Runnable onInit, Runnable onExecute, java.util.function.Consumer<Boolean> onEnd, BooleanSupplier isFinished, SubsystemBase... requirements) {}
}
