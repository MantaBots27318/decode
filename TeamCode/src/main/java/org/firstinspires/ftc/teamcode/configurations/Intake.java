/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the decode robot first version (6th december)
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class Intake extends Configuration {

    protected void initialize(){

        mVersion = Version.INTAKE;

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",true));
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",true));
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false));
        mMotors.put("back-right-wheel",new ConfMotor("backRight",false));

        /* IMUs configuration */
        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        mImus.put("pinpoint", new ConfImu("pinpoint"));
        mImus.get("pinpoint").setPar(3.5, false);
        mImus.get("pinpoint").setPerp(-5.625, true);

        /* Intake configuration */
        mMotors.put("intake-brushes",new ConfMotor("intakeBrushes",false));
        mMotors.put("intake-belts",new ConfMotor("intakeBrushes",false));

        /* Transfer configuration */
        mMotors.put("transfer-wheels",new ConfMotor("transferWheels",true));

        /* Outtake configuration */
        mMotors.put("outtake-wheels", new ConfMotor("outtakeWheels", true));
        mServos.put("outtake-lever-arm", new ConfServo("outtakeLeverArm", false));
        mServos.get("outtake-lever-arm").addPosition("open", 1.0);
        mServos.get("outtake-lever-arm").addPosition("shoot", 0.375);
        mServos.get("outtake-lever-arm").addPosition("intake", 0.85);
        mServos.get("outtake-lever-arm").addPosition("lock", 0.42);
        mServos.get("outtake-lever-arm").addPosition("next", 0.55);

        /* Camera servo configuration */
        mServos.put("camera", new ConfServo("camera", false));
        mServos.get("camera").addPosition("tag",0.89);
        mServos.get("camera").addPosition("ball",0.6);

        /* Limelight configuration */
        mLimelights.put("limelight", new ConfLimelight("limelight"));
        mLimelights.get("limelight").addPipeline("balls-detector",1);
        mLimelights.get("limelight").addPipeline("localizer",0);

    }

}