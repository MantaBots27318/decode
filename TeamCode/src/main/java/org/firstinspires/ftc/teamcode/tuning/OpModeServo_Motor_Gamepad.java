package org.firstinspires.ftc.teamcode.tuning;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Driving;
import org.firstinspires.ftc.teamcode.configurations.Configuration;

@TeleOp
public class OpModeServo_Motor_Gamepad extends OpMode {

    Servo  mServo;
    DcMotor mDcMotor1;
    DcMotor mDcMotor2;
    Driving mDriving;

    public void init(){
        mDriving = new Driving();
        mServo = hardwareMap.get(Servo.class, "Servo1");
        mDcMotor1 = hardwareMap.get(DcMotor.class,"DcMotor1");
        mDcMotor2 = hardwareMap.get(DcMotor.class,"DcMotor2");
        mDriving.setHW(Configuration.s_Current,hardwareMap,telemetry,gamepad2);

    }
    public void loop(){
        mDriving.control();
        mServo.setPosition(gamepad1.right_stick_x);
        mDcMotor1.setPower(gamepad1.left_stick_y);
        mDcMotor2.setPower(gamepad1.left_stick_x*(-1));
    }
}