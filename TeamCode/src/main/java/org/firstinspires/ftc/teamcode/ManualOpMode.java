package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.pose.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;

@TeleOp
public class ManualOpMode extends LinearOpMode {

    Logger      mLogger;

    Alliance    mAlliance;

    Robot       mRobot;

    Path        mPath;

    Controller  mGamepad1;
    Controller  mGamepad2;

    public void runOpMode() throws InterruptedException {

        mLogger         = new Logger(telemetry, FtcDashboard.getInstance(),"manual");

        mAlliance = Alliance.NONE;
        Double alliance_value = Configuration.s_Current.retrieve("alliance");
        if(alliance_value == null) { alliance_value = Alliance.RED.getValue(); }
        if(Math.abs(alliance_value - Alliance.RED.getValue()) < 0.01)  { mAlliance = Alliance.RED;}
        if(Math.abs(alliance_value - Alliance.BLUE.getValue()) < 0.01) { mAlliance = Alliance.BLUE;}

        mGamepad1 = new Controller(gamepad1,mLogger);
        mGamepad2 = new Controller(gamepad2,mLogger);

        mPath = new Path(mLogger);
        mPath.initialize(mAlliance, true);

        mRobot = new Robot();
        mRobot.setHW(Configuration.s_Current,hardwareMap,mLogger, mGamepad1, mGamepad2, mPath);

        mLogger.info("ALL : " +  mAlliance);
        mLogger.update();

        waitForStart();
        while (opModeIsActive()){

            try {
                mRobot.control();
                mRobot.loop();
                mLogger.update();
            } catch (Exception e) {
                mLogger.error("LOOP error : " + e.getMessage());
                mLogger.update();
            }
        }

        mRobot.close();
        Configuration.s_Current.reinit();
    }


}
