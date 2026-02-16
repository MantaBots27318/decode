/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the decode robot first version (6th december)
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class V3 extends Configuration {

    protected void initialize(){

        mVersion = Version.V3;

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",true));
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",true));
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",false));
        mMotors.put("back-right-wheel",new ConfMotor("backRight",false));

        /* IMUs configuration */
        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        /* Intake configuration */
        mMotors.put("intake-brushes",new ConfMotor("intakeBrushes",true));
        mMotors.put("intake-belts",new ConfMotor("intakeBrushes",true));

        /* Transfer configuration */
        mMotors.put("transfer-wheels",new ConfMotor("transferWheels",true));

        /* Turret configuration */

    }

}