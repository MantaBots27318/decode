/* -------------------------------------------------------
   Copyright (c) [2024] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the robot second version (18th of january)
   ------------------------------------------------------- */

package org.firstinspires.ftc.teamcode.configurations;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class V1 extends Configuration {

    protected void initialize(){

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",true));          // EH Motor 3
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));            // EH Motor 2
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false));         // CH Motor 2
        mMotors.put("back-right-wheel",new ConfMotor("backRight",false));           // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        mImus.put("pinpoint", new ConfImu("pinpoint")); // EH I2C 3

        mImus.get("pinpoint").setPar(3.5, false);
        mImus.get("pinpoint").setPerp(-5.625, true);


        /* Intake configuration */
        mMotors.put("intake-brushes",new ConfMotor("intakeBrushes",false));        // EH Motor 0

        /* Outtake configuration */
        mMotors.put("outtake-wheels", new ConfMotor("outtakeWheels", false));      // CH Motor 1
        mServos.put("outtake-lever-arm", new ConfServo("outtakeLeverArm", false)); // EH Servo 0

        /* Camera configuration */
        mServos.put("camera", new ConfServo("camera", false));                     // CH Servo 0
        mLimelights.put("limelight", new ConfLimelight("limelight"));

        /* Outtake servos reference positions */
        mServos.get("outtake-lever-arm").addPosition("open", 1.0);
        mServos.get("outtake-lever-arm").addPosition("shoot", 0.375);
        mServos.get("outtake-lever-arm").addPosition("next", 0.5);

        /* Camera servo reference position */
        mServos.get("camera").addPosition("tag",0.89);
        mServos.get("camera").addPosition("ball",0.6);

        /* Limelight configuration */
        mLimelights.get("limelight").addPipeline("balls-detector",1);
        mLimelights.get("limelight").addPipeline("localizer",0);

    }

}