package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

@Config
@TeleOp
public class ShootingPositionTest extends LinearOpMode{
    public static boolean SHOOTPOSITION = false;
    Driving         mDriving;


    public void runOpMode() throws InterruptedException {
        mDriving = new Driving();
        mDriving.setHW(Configuration.s_Current,hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1);
        waitForStart();
        while(opModeIsActive()) {

            if(SHOOTPOSITION == true) {
                mDriving.shootPosition();
                SHOOTPOSITION = false;
            }
        }
    }


}
