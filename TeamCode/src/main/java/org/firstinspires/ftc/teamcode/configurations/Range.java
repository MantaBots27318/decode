package org.firstinspires.ftc.teamcode.configurations;


public enum Range {
    FAR(0.0),
    CLOSE(1.0),
    NONE(2.0);

    final double mValue;

    Range(double value) {
        mValue = value;
    }

    public Double getValue() {
        return mValue;
    }
}
