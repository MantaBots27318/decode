package org.firstinspires.ftc.robotcore.external.navigation;

public class Pose3D {

    public static class Position {
        public final double x, y, z;
        public Position(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }
    }

    public static class Orientation {
        private final double yawDegrees;
        public Orientation(double yawDegrees) { this.yawDegrees = yawDegrees; }
        public double getYaw() { return yawDegrees; }
    }

    private final Position position;
    private final Orientation orientation;

    public Pose3D(double x, double y, double z, double yawDegrees) {
        this.position = new Position(x, y, z);
        this.orientation = new Orientation(yawDegrees);
    }

    public Position getPosition() { return position; }
    public Orientation getOrientation() { return orientation; }
}