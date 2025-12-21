package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class speedTest extends OpMode {

    DcMotorEx motor1;

    public void init(){
        FtcDashboard.getInstance().getTelemetry().addLine("here1");
        motor1 = hardwareMap.get(DcMotorEx.class,"intakeBrushes");
        FtcDashboard.getInstance().getTelemetry().addLine("here2");
        FtcDashboard.getInstance().getTelemetry().update();
    }

    public void loop(){
        motor1.setPower(1.0);
        double intake_velocity = motor1.getVelocity();
        FtcDashboard.getInstance().getTelemetry().addData("speed" , intake_velocity);
        FtcDashboard.getInstance().getTelemetry().update();
    }


}
