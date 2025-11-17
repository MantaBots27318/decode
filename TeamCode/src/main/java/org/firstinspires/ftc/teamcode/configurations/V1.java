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
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));      // CH Motor 0
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));        // CH Motor 1
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",true));     // CH Motor 2
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));       // CH Motor 3

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        mImus.put("pinpoint", new ConfImu("pinpoint"));                                                // EH I2C 3

        /* Intake configuration */
        mMotors.put("intake",new ConfMotor("intake",false));

        /* Outtake configuration */
        mMotors.put("outtake-wheels", new ConfMotor("outtakeWheels", false));
        mServos.put("outtake-lever-arm", new ConfServo("outtakeLeverArm", false));

        /* Camera configuration */
        mServos.put("camera", new ConfServo("camera", false));

        /* Outtake servos reference positions */
        mServos.get("outtake-wheel").addPosition("open", 0.8);
        mServos.get("outtake-wheel").addPosition("shoot", 0.4);
        mServos.get("outtake-wheel").addPosition("next", 0.6);

        /* Camera servo reference position */
        mServos.get("camera").addPosition("tag",0.3);
        mServos.get("camera").addPosition("ball",0.18);


    }

}