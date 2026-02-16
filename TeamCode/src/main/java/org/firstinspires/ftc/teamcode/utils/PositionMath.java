package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PositionMath {
    Vision mVision;
    Logger mLogger;



    public Pose3D limelightRobotPose(double angle_turret){
        Pose3D position = mVision.getPosition();
        double radius_turret = 0.0;

        }

    public static Pose2d getRobotPoseFromLimelight(
            Pose2d limelightFieldPose,
            Pose2d limelightRobotPose
    ) {
        // Robot heading on field
        double robotHeadingField =
                limelightFieldPose.heading.toDouble() - limelightRobotPose.heading.toDouble();

        // Rotate limelight offset into field frame
        double cos = Math.cos(robotHeadingField);
        double sin = Math.sin(robotHeadingField);

        double offsetXField =
                limelightRobotPose.position.x * cos - limelightRobotPose.position.y * sin;

        double offsetYField =
                limelightRobotPose.position.x * sin + limelightRobotPose.position.y * cos;

        // Robot center position on field
        double robotXField = limelightFieldPose.position.x - offsetXField;
        double robotYField = limelightFieldPose.position.y - offsetYField;

        return new Pose2d(robotXField, robotYField, robotHeadingField);
    }
}