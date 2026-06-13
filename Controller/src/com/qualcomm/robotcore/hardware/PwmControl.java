package com.qualcomm.robotcore.hardware;
public interface PwmControl {
    class PwmRange {
        public final double usPulseLower, usPulseUpper;
        public PwmRange(double usPulseLower, double usPulseUpper) {
            this.usPulseLower = usPulseLower;
            this.usPulseUpper = usPulseUpper;
        }
    }
    void setPwmRange(PwmRange range);
    PwmRange getPwmRange();
    void setPwmEnable();
    void setPwmDisable();
    boolean isPwmEnabled();
}
