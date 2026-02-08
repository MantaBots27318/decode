/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Configuration for the into-the-deep robot
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.configurations;

/* Qualcomm includes */
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LastYear extends Configuration {

    protected void initialize(){

        mVersion = Version.NONE;

        /* Moving configuration : Positive power makes wheel go forward */
        mMotors.put("front-left-wheel",new ConfMotor("frontLeft",false));
        mMotors.put("back-left-wheel",new ConfMotor("backLeft",false));
        mMotors.put("front-right-wheel",new ConfMotor("frontRight",true));
        mMotors.put("back-right-wheel",new ConfMotor("backRight",true));

        mImus.put("built-in", new ConfImu("imu", RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        mImus.put("pinpoint", new ConfImu("pinpoint"));

        /* Pinpoint configuration */
        mImus.get("pinpoint").setPar(-5.22, false);
        mImus.get("pinpoint").setPerp(0.0, false);

        /* Intake configuration */
        mMotors.put("intake-slides",new ConfMotor(
                "intakeSlidesLeft",false, false,
                "intakeSlidesRight",true, false));
        mServos.put("intake-arm-pitch", new ConfServo(
                "intakeArmPitchLeft", false,
                "intakeArmPitchRight", true
        ));
        mServos.put("intake-elbow-pitch", new ConfServo("intakeElbowPitch", false));
        mServos.put("intake-wrist-roll", new ConfServo("intakeWristRoll", false));
        mServos.put("intake-claw", new ConfServo("intakeClaw", false));

        /* Outtake configuration */
        mMotors.put("outtake-slides",new ConfMotor(
                "outtakeSlidesLeft",true, false,
                       "outtakeSlidesRight",false, false));

        mServos.put("outtake-wrist-roll", new ConfServo("outtakeWristRoll", false));
        mServos.put("outtake-claw", new ConfServo("outtakeClaw", false));
        mServos.put("outtake-elbow-pitch", new ConfServo(
                "outtakeElbowPitchLeft", false,
                "outtakeElbowPitchRight", true)
        );

        mLimelights.put("limelight", new ConfLimelight("limelight"));

        mMotors.get("intake-slides").addPosition("min",0 );
        mMotors.get("intake-slides").addPosition("transfer",167 );
        mMotors.get("intake-slides").addPosition("retracted",250 );
        mMotors.get("intake-slides").addPosition("init",300 );
        mMotors.get("intake-slides").addPosition("autonomous",280 );
        mMotors.get("intake-slides").addPosition("max",315 );

        /* Outtake motors reference positions */
        mMotors.get("outtake-slides").addPosition("min",0 );
        mMotors.get("outtake-slides").addPosition("transfer",0 );
        mMotors.get("outtake-slides").addPosition("retracted",1300 );
        mMotors.get("outtake-slides").addPosition("max",3726 );
        mMotors.get("outtake-slides").addPosition("highSubmersibleUnder",690 );
        mMotors.get("outtake-slides").addPosition("highSubmersibleOver",845 );
        mMotors.get("outtake-slides").addPosition("ascend",710 );

        /* Intake servos reference positions */
        mServos.get("intake-arm-pitch").addPosition("transfer", 0.97);
        mServos.get("intake-arm-pitch").addPosition("overSub", 0.6);

        mServos.get("intake-arm-pitch").addPosition("look", 0.44 );
        mServos.get("intake-arm-pitch").addPosition("grab", 0.39);
        mServos.get("intake-arm-pitch").addPosition("off", 1.0);

        mServos.get("intake-elbow-pitch").addPosition("transfer", 0.15);
        mServos.get("intake-elbow-pitch").addPosition("grab", 0.66);
        mServos.get("intake-elbow-pitch").addPosition("look", 0.68);
        mServos.get("intake-elbow-pitch").addPosition("overSub", 0.71);
        mServos.get("intake-elbow-pitch").addPosition("off", 0.66);
        mServos.get("intake-elbow-pitch").addPosition("topView", 0.45);
        mServos.get("intake-elbow-pitch").addPosition("visionStart", 0.51);

        mServos.get("intake-wrist-roll").addPosition("-2", 0.27);
        mServos.get("intake-wrist-roll").addPosition("-1", 0.335);
        mServos.get("intake-wrist-roll").addPosition("0", 0.405);
        mServos.get("intake-wrist-roll").addPosition("1", 0.47);
        mServos.get("intake-wrist-roll").addPosition("2", 0.54);
        mServos.get("intake-wrist-roll").addPosition("3", 0.605);
        mServos.get("intake-wrist-roll").addPosition("4", 0.675);
        mServos.get("intake-wrist-roll").addPosition("5", 0.74);
        mServos.get("intake-wrist-roll").addPosition("6", 0.82);

        mServos.get("intake-claw").addPosition("closed", 1.0);
        mServos.get("intake-claw").addPosition("microrelease", 0.9);
        mServos.get("intake-claw").addPosition("open", 0.62);

        /* Outtake servos reference positions */
        mServos.get("outtake-wrist-roll").addPosition("-2", 0.0);
        mServos.get("outtake-wrist-roll").addPosition("-1", 0.0675);
        mServos.get("outtake-wrist-roll").addPosition("0", 0.135);
        mServos.get("outtake-wrist-roll").addPosition("1", 0.2075);
        mServos.get("outtake-wrist-roll").addPosition("2", 0.28);
        mServos.get("outtake-wrist-roll").addPosition("3", 0.35);
        mServos.get("outtake-wrist-roll").addPosition("4", 0.42);
        mServos.get("outtake-wrist-roll").addPosition("5", 0.465);
        mServos.get("outtake-wrist-roll").addPosition("6",  0.51);

        mServos.get("outtake-claw").addPosition("ultraclosed", 0.75);
        mServos.get("outtake-claw").addPosition("closed", 0.73);
        mServos.get("outtake-claw").addPosition("microrelease", 0.70);
        mServos.get("outtake-claw").addPosition("open", 0.36);


        mServos.get("outtake-elbow-pitch").addPosition("transfer", 0.11);//0.11
        mServos.get("outtake-elbow-pitch").addPosition("drop", 0.05);
        mServos.get("outtake-elbow-pitch").addPosition("off", 0.08);
        mServos.get("outtake-elbow-pitch").addPosition("vertical", 0.085);
        mServos.get("outtake-elbow-pitch").addPosition("specimen", 0.011);
        mServos.get("outtake-elbow-pitch").addPosition("specimen2", 0.025);

        mLimelights.get("limelight").addPipeline("balls-detector",2);
        mLimelights.get("limelight").addPipeline("localizer",0);

    }

}