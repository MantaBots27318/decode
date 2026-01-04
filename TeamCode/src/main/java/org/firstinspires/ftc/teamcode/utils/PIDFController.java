package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.ValueProvider;

public class PIDFController {

    // Gains
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    // State
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = Double.NaN;

    // Anti-windup
    private final double integralMin;
    private final double integralMax;

    public PIDFController(
            double kP,
            double kI,
            double kD,
            double kF,
            double integralMin,
            double integralMax
    ) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integralMin = integralMin;
        this.integralMax = integralMax;
    }

    /**
     * @param target desired setpoint
     * @param measurement current measured value
     * @param currentTimeSeconds timestamp in seconds (System.nanoTime()/1e9)
     */
    public double update(double target, double measurement, double currentTimeSeconds) {

        double error = target - measurement;

        // First call: no derivative or integral yet
        if (Double.isNaN(lastTime)) {
            lastTime = currentTimeSeconds;
            lastError = error;
            return kP * error + kF * target;
        }

        double dt = currentTimeSeconds - lastTime;
        if (dt <= 0) {
            // Defensive: time didn't advance
            return kP * error + kF * target;
        }

        // Integral
        integral += error * dt;
        integral = clamp(integral, integralMin, integralMax);

        // Derivative
        double derivative = (error - lastError) / dt;

        // PIDF output
        double output =
                kP * error +
                        kI * integral +
                        kD * derivative +
                        kF * target;

        // Update state
        lastError = error;
        lastTime = currentTimeSeconds;

        return output;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastTime = Double.NaN;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static class PIDFProvider implements ValueProvider<Double> {
        Double mValue;
        public PIDFProvider( double Value) {
            mValue = Value;
        }
        @Override
        public Double get()              { return mValue;  }
        @Override
        public void set(Double Value)    { mValue = Value; }
    }
}
