/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the decode robot first version (6th december)
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/* AcmeRobotics includes */
import com.acmerobotics.roadrunner.Pose2d;

public class V3 extends Configuration {

    protected void initialize(){

        mVersion = Version.V3;

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",true));
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));

        /* IMUs configuration */
        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        mImus.put("pinpoint", new ConfImu("pinpoint"));
        mImus.get("pinpoint").setPar(1.6, false);
        mImus.get("pinpoint").setPerp(-(6.0+14.0/16), true);

        /* Intake configuration */
        mMotors.put("intake-wheels",new ConfMotor("intakeWheels",true));
        mMotors.put("guiding-wheels",new ConfMotor("guidingWheels",true));

        /* Transfer configuration */
        mServos.put("transfer-servo", new ConfServo("transferServo", false));
        mServos.get("transfer-servo").addPosition("block", 0.58);
        mServos.get("transfer-servo").addPosition("let", 0.5);
        mServos.get("transfer-servo").addPosition("down", 0.61);

        /* Limelight configuration */
        mLimelights.put("limelight", new ConfLimelight("limelight"));
        mLimelights.get("limelight").addPipeline("balls-detector",1);
        mLimelights.get("limelight").addPipeline("localizer",0);

        /* Outtake configuration */
        mMotors.put("outtake-wheels",new ConfMotor(
                "outtakeWheelsLeft",true, false,
                "outtakeWheelsRight",false, false));

        mServos.put("turret-rotation", new ConfServo(
                "turretRotationLeft", false,
                "turretRotationRight", false));
        mServos.get("turret-rotation").addPosition("min", 0.35);
        mServos.get("turret-rotation").addPosition("max", 0.63);
        mServos.put("turret-hood", new ConfServo("turretHood", false));
        mServos.get("turret-hood").addPosition("min", 0.454);
        mServos.get("turret-hood").addPosition("max", 0.58);

        // Turntable encoder is plugged on the intake wheels motor
        mEncoders.put("turret-rotation", new ConfEncoder("intakeWheels",false));

        /* Components relative positions in robot reference */
        mPositions.put("limelight-rotation-radius", new Pose2d(6,6,0));
        mPositions.put("turret", new Pose2d(-10,0,0));

    }

}