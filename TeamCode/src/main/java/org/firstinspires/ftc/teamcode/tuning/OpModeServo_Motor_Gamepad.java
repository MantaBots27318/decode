package org.firstinspires.ftc.teamcode.tuning;

/* Qualcomm includes */
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp
public class OpModeServo_Motor_Gamepad extends OpMode {

    Servo  mServo;
    DcMotor mDcMotor1;
    DcMotor mDcMotor2;
    Driving mDriving;
    Vision mVision;

    public void init(){
        mVision  = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mDriving = new Driving();
        mServo = hardwareMap.get(Servo.class, "Servo1");
        mDcMotor1 = hardwareMap.get(DcMotor.class,"DcMotor1");
        mDcMotor2 = hardwareMap.get(DcMotor.class,"DcMotor2");
        mDriving.setHW(Configuration.s_Current,hardwareMap, FtcDashboard.getInstance().getTelemetry(), gamepad1,mVision);
        mVision.initialize();
    }
    public void loop(){

        mDriving.control();
        mServo.setPosition(gamepad1.right_stick_x+0.1);
        mDcMotor1.setPower(gamepad1.left_stick_y);
        mDcMotor2.setPower(gamepad1.left_stick_x*(-1));
    }
    @Override
    public void stop(){
        mVision.close();
    }

}