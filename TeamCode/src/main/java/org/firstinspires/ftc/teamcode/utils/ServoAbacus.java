package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoAbacus {

    // Flywheel speed tiers (ticks/s) — must be sorted ascending.
    // Add more tiers as you measure them on the field.
    private static final double[] sSpeeds = {1450, 1550, 1650, 1750 };

    // For each speed tier: { distance (inches), hood servo position }
    // Distances must be sorted ascending within each tier.
    // Replace 0.0 placeholder values with measured data.
    private static final double[][][] sReferenceTable = {
        // Speed tier: 1450 ticks/s (low battery)

        {
            {  0, 0.8 },
            { 50, 0.8 },
            { 62, 0.8 },
            {129, 0.8 },
            
        },

        // Speed tier: 1550 ticks/s (low battery)
        {
            {  42, 0.8 },
            { 52, 0.8 },
            { 62, 0.8 },
            {70, 0.8 },
            {80, 0.9 },
        },
        // Speed tier: 1650 ticks/s
        {
            {  41, 0.75 },
            { 53, 0.65 },
            { 60, 0.65 },
            {72, 0.6 },
            {80, 0.58 },
        },
        {
                {0, 0.66},
                {49, 0.66},
                {50, 0.66},
                {57, 0.64},
                {70, 0.66},
                {90, 0.68},
                {99, 0.46 + 0.23},
                {129, 0.35 + 0.23},
                {139, 0.35 + 0.23},
                {500, 0.42 + 0.23}
        }
    };

    /**
     * Returns the hood servo position for a given distance and actual flywheel speed.
     * Bilinearly interpolates across the speed tiers and distance points.
     */
    public static double getPosition(double distance, double flywheelSpeed) {

        // Clamp speed to table range
        if (flywheelSpeed <= sSpeeds[0]) {
            return interpolateDistance(sReferenceTable[0], distance);
        }
        if (flywheelSpeed >= sSpeeds[sSpeeds.length - 1]) {
            return interpolateDistance(sReferenceTable[sSpeeds.length - 1], distance);
        }

        // Find bracketing speed tiers
        int lo = 0;
        for (int i = 0; i < sSpeeds.length - 1; i++) {
            if (flywheelSpeed >= sSpeeds[i] && flywheelSpeed <= sSpeeds[i + 1]) {
                lo = i;
                break;
            }
        }
        int hi = lo + 1;

        double posLo = interpolateDistance(sReferenceTable[lo], distance);
        double posHi = interpolateDistance(sReferenceTable[hi], distance);

        // Interpolate between the two speed tiers
        double t = (flywheelSpeed - sSpeeds[lo]) / (sSpeeds[hi] - sSpeeds[lo]);
        return posLo + t * (posHi - posLo);
    }

    /** Backward-compatible single-argument version — assumes maximum flywheel speed. */
    public static double getPosition(double distance) {
        return getPosition(distance, sSpeeds[sSpeeds.length - 1]);
    }

    private static double interpolateDistance(double[][] table, double distance) {

        if (distance <= table[0][0]) {
            return table[0][1];
        }
        if (distance >= table[table.length - 1][0]) {
            return table[table.length - 1][1];
        }

        for (int i = 0; i < table.length - 1; i++) {
            double d1 = table[i][0];
            double p1 = table[i][1];
            double d2 = table[i + 1][0];
            double p2 = table[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                return p1 + (distance - d1) * (p2 - p1) / (d2 - d1);
            }
        }

        return table[table.length - 1][1];
    }
}