package org.firstinspires.ftc.teamcode.test;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp
public class RollingTest extends OpMode {

    public static double POWER = -1.0;

    DcMotor mMotor1;
    DcMotor mMotor2;

    public void init(){
        mMotor1 = hardwareMap.get(DcMotor.class,"motor1");
        mMotor2 = hardwareMap.get(DcMotor.class,"motor2");
    }
    public void loop(){
        mMotor1.setPower(POWER);
        mMotor2.setPower(POWER);
    }

}