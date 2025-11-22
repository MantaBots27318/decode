package org.firstinspires.ftc.teamcode.configurations;

public enum Alliance {
    Blue(0.0),
    Red(1.0),
    None(2.0);

    final Double mValue;

    Alliance(Double value) {
        mValue = value;
    }

    public Double getValue() { return mValue; }
}