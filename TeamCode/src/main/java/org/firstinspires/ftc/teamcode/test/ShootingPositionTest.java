package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.Poses;
import org.firstinspires.ftc.teamcode.configurations.Range;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
@TeleOp
public class ShootingPositionTest extends LinearOpMode{

    public static boolean   SHOOTPOSITION = false;
    public static Alliance  ALLIANCE = Alliance.BLUE;

    Driving         mDriving;
    Vision          mVision;
    Poses           mPoses;

    Controller      mGamepad;

    public void runOpMode() throws InterruptedException {

        mGamepad = new Controller(gamepad1, FtcDashboard.getInstance().getTelemetry());
        mPoses   = new Poses(FtcDashboard.getInstance().getTelemetry());
        mPoses.initialize(ALLIANCE, Vision.Pattern.GPP,true);

        mDriving = new Driving();
        mVision  = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);

        mDriving.setHW(Configuration.s_Current,hardwareMap, FtcDashboard.getInstance().getTelemetry(), mGamepad,mVision, mPoses);
        mVision.initialize();
        waitForStart();
        while(opModeIsActive()) {

            if(SHOOTPOSITION == true) {
                mDriving.shootPosition(Range.FAR);
                SHOOTPOSITION = false;
            }

        }
        mVision.close();
    }


}
