package com.qualcomm.hardware.rev;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
public class Rev2mDistanceSensor implements DistanceSensor {
    @Override public double getDistance(DistanceUnit unit) { return 0; }
    @Override public Manufacturer getManufacturer() { return Manufacturer.Unknown; }
    @Override public String getDeviceName() { return "Rev2mDistanceSensor"; }
    @Override public String getConnectionInfo() { return ""; }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() {}
    @Override public void close() {}
    public void initialize() {}
}
