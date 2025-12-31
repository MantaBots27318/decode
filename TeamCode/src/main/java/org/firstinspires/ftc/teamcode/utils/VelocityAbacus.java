package org.firstinspires.ftc.teamcode.utils;

public class VelocityAbacus {

    // Measured data points (distance in meters, power from 0.0 to 1.0)
    // IMPORTANT: distances must be sorted in increasing order
    private static final double[][] sReferenceTable = {
            {1.0, 0.25},
            {2.0, 0.35},
            {3.0, 0.45},
            {4.0, 0.55},
            {5.0, 0.65}
    };

    /**
     * Predicts the power needed to shoot from a given distance.
     *
     * @param distance Distance to the target
     * @return Interpolated power value
     */
    public static double getVelocity(double distance) {

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

        // Should never happen
        return result;
    }
}
