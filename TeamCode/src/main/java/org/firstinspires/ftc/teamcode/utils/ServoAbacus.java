package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ServoAbacus {

    // Measured data points (distance in meters, power from 0.0 to 1.0)
    // IMPORTANT: distances must be sorted in increasing order
    private static final double[][] sReferenceTable = {
            {0,0.43},
            {49,0.43},
            {50,0.43},
            {57,0.41},//
            {70,0.43},//
            {90,0.45},//
            {99,0.46},//
            {129, 0.46},
            {139, 0.42},
            {500, 0.42}
    };

    /**
     * Predicts the power needed to shoot from a given distance.
     *
     * @param distance Distance to the target
     * @return Interpolated power value
     */
    public static double getPosition(double distance) {

        double result = 0;

        // If below minimum distance
        if (distance <= sReferenceTable[0][0]) {
            result = sReferenceTable[0][1];
        }

        // If above maximum distance
        if (distance >= sReferenceTable[sReferenceTable.length - 1][0]) {
            result = sReferenceTable[sReferenceTable.length - 1][1];
        }

        // Find the two points surrounding the distance
        for (int i = 0; i < sReferenceTable.length - 1; i++) {
            double d1 = sReferenceTable[i][0];
            double p1 = sReferenceTable[i][1];
            double d2 = sReferenceTable[i + 1][0];
            double p2 = sReferenceTable[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                // Linear interpolation
                result =  p1 + (distance - d1) * (p2 - p1) / (d2 - d1);
            }
        }

        return result;
    }
}
