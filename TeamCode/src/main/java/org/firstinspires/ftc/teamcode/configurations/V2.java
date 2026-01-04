/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the decode robot first version (30th december)
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class V2 extends Configuration {

    protected void initialize(){

        mVersion = Version.V2;

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",true));
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false));
        mMotors.put("back-right-wheel",new ConfMotor("backRight",false));

        /* IMUs configuration */
        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        mImus.put("pinpoint", new ConfImu("pinpoint"));
        mImus.get("pinpoint").setPar(-3.1, false);
        mImus.get("pinpoint").setPerp(-8.0, true);

        /* Intake configuration */
        mMotors.put("intake-belts",new ConfMotor("intakeBelts",false));

        /* Outtake configuration */
        mMotors.put("outtake-wheels",new ConfMotor(
                "outtakeWheelsLeft",false, false,
                "outtakeWheelsRight",true, false));
        mServos.put("outtake-lever-arm", new ConfServo("outtakeLeverArm", false));
        mServos.get("outtake-lever-arm").addPosition("lock", 0.0);
        mServos.get("outtake-lever-arm").addPosition("shoot", 0.3);

        /* Camera servo configuration */
        mServos.put("camera", new ConfServo("camera", false));
        mServos.get("camera").addPosition("tag",0.89);
        mServos.get("camera").addPosition("ball",0.6);

        /* Limelight configuration */
        mLimelights.put("limelight", new ConfLimelight("limelight"));
        mLimelights.get("limelight").addPipeline("balls-detector",1);
        mLimelights.get("limelight").addPipeline("localizer",0);

        /* Leds configuration */
        mLeds.put("tracking", new ConfLed("trackingLeft","trackingRight"));
        mLeds.put("engaged", new ConfLed("engagedLeft","engagedRight"));

    }

}