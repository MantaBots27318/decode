package com.qualcomm.robotcore.hardware;
public interface LED extends HardwareDevice {
    void enable(boolean enable);
    boolean isEnabled();
}
