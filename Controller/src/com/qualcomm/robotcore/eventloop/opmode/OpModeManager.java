package com.qualcomm.robotcore.eventloop.opmode;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
public interface OpModeManager {
    void register(String name, Class<? extends OpMode> opMode);
    void register(OpModeMeta meta, Object opModeInstance);
    void register(OpModeMeta meta, Class<?> opModeClass);
}
