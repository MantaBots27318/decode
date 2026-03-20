package org.firstinspires.ftc.teamcode.pose;

/* Acmerobotics includes */
import com.acmerobotics.roadrunner.Pose2d;

public interface Posable {

    Pose2d  getFTCPosition();

    void    setFTCPosition(Pose2d position);

    static Pose2d derivePose(
            Pose2d referenceFTCPose,
            Pose2d referenceRobotPose
    ) {
        // Robot heading on field
        double robotHeadingField =
                referenceFTCPose.heading.toDouble() - referenceRobotPose.heading.toDouble();

        // Rotate limelight offset into field frame
        double cos = Math.cos(robotHeadingField);
        double sin = Math.sin(robotHeadingField);

        double offsetXField =
                referenceRobotPose.position.x * cos - referenceRobotPose.position.y * sin;

        double offsetYField =
                referenceRobotPose.position.x * sin + referenceRobotPose.position.y * cos;

        // Robot center position on field
        double robotXField = referenceFTCPose.position.x - offsetXField;
        double robotYField = referenceFTCPose.position.y - offsetYField;

        return new Pose2d(robotXField, robotYField, robotHeadingField);
    }

}
