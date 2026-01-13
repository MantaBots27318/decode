package org.firstinspires.ftc.teamcode.tuning;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config

@TeleOp(name="OpModeServo_Motor", group="Tuning")
public class OpModeServo_Motor extends OpMode {

    Servo  mServo;
    DcMotor mDcMotor1;
    DcMotor mDcMotor2;

    public static double SERVO = 1;
    public static double MOTOR1 = 0;
    public static double MOTOR2 = 0;

    public void init(){
        mServo = hardwareMap.get(Servo.class, "camera");
        mDcMotor1 = hardwareMap.get(DcMotor.class,"intakeBeltsFront");
        mDcMotor2 = hardwareMap.get(DcMotor.class,"intakeBeltsBack");
    }
    public void loop(){
        mServo.setPosition(SERVO);
        mDcMotor1.setPower(MOTOR1);
        mDcMotor2.setPower(MOTOR2*(-1));
    }

}