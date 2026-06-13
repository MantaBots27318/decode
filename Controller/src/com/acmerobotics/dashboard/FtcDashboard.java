package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.ValueProvider;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.Consumer;

public class FtcDashboard {
    private static FtcDashboard instance;

    public interface ConfigRoot {
        void putVariable(String name, Object variable);
    }

    private FtcDashboard(){}

    public static FtcDashboard getInstance() {
        if (instance == null) {
            instance = new FtcDashboard();
        }
        return instance;
    }

    public void sendTelemetryPacket(TelemetryPacket telemetryPacket) {}

    public Telemetry getTelemetry(){
        return null;
    }

    public void setTelemetryTransmissionInterval(int telemetryTransmissionInterval){}

    public <T> void addConfigVariable(String className, String name, ValueProvider<T> provider) {}

    public void updateConfig() {}

    public void withConfigRoot(Consumer<ConfigRoot> consumer) {}
}