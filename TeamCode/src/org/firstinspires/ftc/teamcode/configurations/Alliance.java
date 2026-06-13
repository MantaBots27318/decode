package org.firstinspires.ftc.teamcode.configurations;


public enum Alliance {
    BLUE(0.0),
    RED(1.0),
    NONE(2.0);

    final double mValue;

    Alliance(double value) {
        mValue = value;
    }

    public Double getValue() {
        return mValue;
    }
}
