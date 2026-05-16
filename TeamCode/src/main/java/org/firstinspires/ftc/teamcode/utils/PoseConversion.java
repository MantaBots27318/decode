package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;

public class PoseConversion {

    // RoadRunner: field-centered, x=forward, y=left, heading=0 faces forward
    // PedroPathing: corner-based, (0,0) at bottom-left corner, 144"x144" field

    public static Pose topedroPose(Pose2d rrPose) {
        double x_rr = rrPose.position.x;
        double y_rr = rrPose.position.y;
        double heading = rrPose.heading.toDouble();

        double x_pedro = 72 - y_rr;
        double y_pedro = x_rr + 72;
        double heading_pedro = heading + Math.PI / 2;

        return new Pose(x_pedro, y_pedro, heading_pedro);
    }

    public static Pose2d toRRPose(Pose pedroPose) {
        double x_pedro = pedroPose.getX();
        double y_pedro = pedroPose.getY();
        double heading_pedro = pedroPose.getHeading();

        double x_rr = y_pedro - 72;
        double y_rr = 72 - x_pedro;
        double heading_rr = heading_pedro - Math.PI / 2;

        return new Pose2d(new Vector2d(x_rr, y_rr), heading_rr);
    }
}