package org.firstinspires.ftc.teamcode.tuning;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config

@TeleOp(name="OpModeServo_Motor", group="Tuning")
public class OpModeServo_Motor extends OpMode {

    ServoImplEx mServo;
    DcMotor mDcMotor1;
    DcMotor mDcMotor2;

    public static double SERVO = 1;
    public static double MOTOR1 = 0;
    public static double MOTOR2 = 0;

    public void init(){
        mServo = hardwareMap.get(ServoImplEx.class,"test");
        mServo.setPwmRange(new PwmControl.PwmRange(500,2500));
        mDcMotor1 = hardwareMap.get(DcMotor.class,"outtakeWheelsLeft");
        mDcMotor2 = hardwareMap.get(DcMotor.class,"outtakeWheelsRight");
    }
    public void loop(){
        mDcMotor1.setPower(MOTOR1);
        mDcMotor2.setPower(MOTOR2);
        mServo.setPosition(SERVO);
    }

}