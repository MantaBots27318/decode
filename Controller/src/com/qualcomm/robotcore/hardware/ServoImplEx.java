package com.qualcomm.robotcore.hardware;
public class ServoImplEx extends ServoImpl implements PwmControl {
    private PwmRange pwmRange = new PwmRange(500, 2500);
    @Override public void setPwmRange(PwmRange range) { this.pwmRange = range; }
    @Override public PwmRange getPwmRange() { return pwmRange; }
    @Override public void setPwmEnable() {}
    @Override public void setPwmDisable() {}
    @Override public boolean isPwmEnabled() { return true; }
}
