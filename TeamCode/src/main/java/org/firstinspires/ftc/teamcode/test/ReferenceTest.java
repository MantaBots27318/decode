/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Calibration test : opmode to test limelight
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.test;

// Java includes

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.pose.Posable;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Vision;


@Config

@TeleOp(name="ReferenceTest", group="Test")
public class ReferenceTest extends LinearOpMode {

    private Vision  mVision;

    private Logger  mLogger;


    @Override
    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"reference-test");

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap,"vision",mLogger);
        mVision.initialize();
        mLogger.update();

        waitForStart();

        while (opModeIsActive()) {

            try {

                Pose3D output = mVision.getPosition();
                if (output != null) {
                       Pose2d robot_position = Posable.derivePose(
                               new Pose2d(
                                       -output.getPosition().x * Path.M_TO_INCHES,
                                       -output.getPosition().y * Path.M_TO_INCHES,
                                       (output.getOrientation().getYaw() + 180) * Math.PI / 180),
                               new Pose2d(7, 2.75, 0));
                        mLogger.metric("X",""+robot_position.position.x);
                    mLogger.metric("Y",""+robot_position.position.y);
                    mLogger.metric("H",""+robot_position.heading.toDouble() / Math.PI * 180);
                }

            }
            catch( Exception e) { mLogger.error(e.getMessage()); }
            mLogger.update();
        }

        mVision.close();
    }

}
