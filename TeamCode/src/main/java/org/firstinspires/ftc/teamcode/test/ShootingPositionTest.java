package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.configurations.Range;
import org.firstinspires.ftc.teamcode.path.Path;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Config
@TeleOp
public class ShootingPositionTest extends LinearOpMode{


    DcMotorEx motor1;
    DcMotorEx motor2;

    public void runOpMode() throws InterruptedException {

        FtcDashboard.getInstance().getTelemetry().addLine("here1");
        motor1 = hardwareMap.get(DcMotorEx.class,"intakeBrushes");
        motor2 = hardwareMap.get(DcMotorEx.class,"outtakeWheels");
        FtcDashboard.getInstance().getTelemetry().addLine("here2");
        FtcDashboard.getInstance().getTelemetry().update();
        waitForStart();
        motor1.setPower(1.0);
        motor2.setPower(0.5);
        double intake_velocity = motor1.getVelocity();
        FtcDashboard.getInstance().getTelemetry().addData("speed" , intake_velocity);
        FtcDashboard.getInstance().getTelemetry().update();
    }


}
