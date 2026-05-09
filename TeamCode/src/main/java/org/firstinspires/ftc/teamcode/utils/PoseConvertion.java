package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;

public class PoseConvertion {
    public static Pose toPedroPose(Pose2d rrPose) {
        double x_rr = rrPose.position.x;   // forward (centered)
        double y_rr = rrPose.position.y;   // left (centered)
        double heading = rrPose.heading.toDouble();

        // Step 1: rotate axes
        double x_pedro = 72-y_rr;
        double y_pedro = x_rr+72;
        double heading_pedro = heading +90;

        // Step 2: shift origin (center → bottom-left)
        x_pedro += 72;
        y_pedro += 72;

        return new Pose(x_pedro, y_pedro, heading_pedro);
    }
}
