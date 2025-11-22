package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
@TeleOp
public class ShootingPositionTest extends LinearOpMode{
    public static boolean SHOOTPOSITION = false;
    Driving         mDriving;
    Vision          mVision;

    public void runOpMode() throws InterruptedException {
        mDriving = new Driving();
        mVision  = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mDriving.setHW(Configuration.s_Current,hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1,mVision);
        mVision.initialize();
        waitForStart();
        while(opModeIsActive()) {

            if(SHOOTPOSITION == true) {
                mDriving.shootPosition();
                SHOOTPOSITION = false;
            }

        }
        mVision.close();
    }


}
