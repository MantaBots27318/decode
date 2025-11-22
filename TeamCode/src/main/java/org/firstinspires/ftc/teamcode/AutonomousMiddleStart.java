/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

// QUALCOMM includes
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// FTCController includes
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// Acmerobotics include
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Local includes
import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Vision;

@Autonomous
public class AutonomousMiddleStart extends LinearOpMode {

    public static final double X_INIT_FTC_INCHES              = - Configuration.FIELD_SIZE_INCHES / 2 + 9;
    public static final double Y_INIT_FTC_INCHES              = 10;
    public static final double ANGLE_INIT_FTC_RADIANS         = 0;

    public static final double X_CALIBRATION_INIT_INCHES      = 74;

    public static final double X_GPP_PATTERN_INIT_INCHES      = 30;
    public static final double X_PGP_PATTERN_INIT_INCHES      = 55;
    public static final double X_PPG_PATTERN_INIT_INCHES      = 70;

    public static final double Y_CALIBRATION_INIT_INCHES_BLUE     = 0;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_BLUE =  Math.PI / 4;
    public static final double Y_PATTERN_INIT_INCHES_BLUE          = 12;
    public static final double ANGLE_PATTERN_INIT_RADIANS_BLUE     = Math.PI / 2;

    public static final double Y_CALIBRATION_INIT_INCHES_RED      = 0;
    public static final double ANGLE_CALIBRATION_INIT_RADIANS_RED = - Math.PI / 4  ;
    public static final double Y_PATTERN_INIT_INCHES_RED         = -12;
    public static final double ANGLE_PATTERN_INIT_RADIANS_RED     = - Math.PI / 2;

    double y_calibration_init_inches ;
    double angle_calibration_init_radians;
    double y_pattern_init_inches;
    double angle_pattern_init_radians;


    Vision          mVision;
    MecanumDrive    mDrive;
    Collecting      mCollecting;

    Vision.Pattern  mPattern;

    Pose2d          mReferencePose;

    SmartTimer      mTimer;
    double          mXOffset        = 0;
    double          mYOffset        = 0;
    double          mAngleOffset    = 0;
    Alliance        mAlliance = Alliance.Blue;


    @Override
    public void runOpMode() throws InterruptedException {




        telemetry.setMsTransmissionInterval(11);

        mCollecting = new Collecting();

        mTimer = new SmartTimer(telemetry);

        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();
        mPattern = Vision.Pattern.NONE;

        mReferencePose = new Pose2d(0, 0, 0);

        mDrive = new MecanumDrive(hardwareMap, mReferencePose);


        boolean dpad_rightWasPressed = false;
        boolean dpad_leftWasPressed = false;
        while (opModeInInit()) {
            if (gamepad1.dpad_right && !dpad_rightWasPressed) {
                mAlliance = Alliance.Red;
            }
            dpad_rightWasPressed = gamepad1.dpad_right;

            // Toggle BLUE
            if (gamepad1.dpad_left && !dpad_leftWasPressed) {
                mAlliance = Alliance.Blue;
            }
            dpad_leftWasPressed = gamepad1.dpad_left;

            // Display menu
            telemetry.addLine("=== TELEOP CONFIG MENU ===");
            telemetry.addLine("Choose Alliance:");
            telemetry.addData("Right", "RED");
            telemetry.addData("Left", "BLUE");
            telemetry.addData("Current Selection", mAlliance);
            telemetry.update();

            Vision.Pattern pattern = mVision.readPattern();
            if (pattern != Vision.Pattern.NONE) {
                mPattern = pattern;
                telemetry.addLine("======= PATTERN =======");
                telemetry.addLine(mPattern.text());
                telemetry.update();
                FtcDashboard.getInstance().getTelemetry().addLine("======= PATTERN =======");
                FtcDashboard.getInstance().getTelemetry().addLine(mPattern.text());
                FtcDashboard.getInstance().getTelemetry().update();
            }

        }
        if (mAlliance == Alliance.Red ){
             y_calibration_init_inches = Y_CALIBRATION_INIT_INCHES_RED ;
             angle_calibration_init_radians = ANGLE_CALIBRATION_INIT_RADIANS_RED ;
             y_pattern_init_inches = Y_PATTERN_INIT_INCHES_RED ;
             angle_pattern_init_radians = ANGLE_PATTERN_INIT_RADIANS_RED ;
        }

        if (mAlliance == Alliance.Blue ){
            y_calibration_init_inches = Y_CALIBRATION_INIT_INCHES_BLUE ;
            angle_calibration_init_radians = ANGLE_CALIBRATION_INIT_RADIANS_BLUE ;
            y_pattern_init_inches = Y_PATTERN_INIT_INCHES_BLUE ;
            angle_pattern_init_radians = ANGLE_PATTERN_INIT_RADIANS_BLUE ;
        }

        telemetry.addLine("======= ACTIONS =======");
        FtcDashboard.getInstance().getTelemetry().addLine("======= ACTIONS =======");

        if (mPattern == Vision.Pattern.GPP) {

            telemetry.addLine("==> GO TO GPP");
            FtcDashboard.getInstance().getTelemetry().addLine("==> GO TO GPP");

            Actions.runBlocking(
                    mDrive.actionBuilder(mReferencePose)
                            .splineTo(new Vector2d(X_GPP_PATTERN_INIT_INCHES - 10,  - 10), -Math.PI / 8)
                            .splineTo(new Vector2d(X_GPP_PATTERN_INIT_INCHES, y_pattern_init_inches ),angle_pattern_init_radians )
                            .build());
        }
        if (mPattern == Vision.Pattern.PGP) {

            telemetry.addLine("==> GO TO PGP");
            telemetry.update();
            FtcDashboard.getInstance().getTelemetry().addLine("==> GO TO PGP");
            FtcDashboard.getInstance().getTelemetry().update();

            Actions.runBlocking(
                    mDrive.actionBuilder(mReferencePose)
                            .splineTo(new Vector2d(X_PGP_PATTERN_INIT_INCHES - 20, y_pattern_init_inches - 20), -Math.PI / 8)
                            .splineTo(new Vector2d(X_PGP_PATTERN_INIT_INCHES, y_pattern_init_inches ), angle_pattern_init_radians )
                            .build());
        }
        if (mPattern == Vision.Pattern.PPG) {

            telemetry.addLine("==> GO TO PPG");
            telemetry.update();
            FtcDashboard.getInstance().getTelemetry().addLine("==> GO TO PGP");
            FtcDashboard.getInstance().getTelemetry().update();

            Actions.runBlocking(
                    mDrive.actionBuilder(mReferencePose)
                            .splineTo(new Vector2d(X_PPG_PATTERN_INIT_INCHES - 20, y_pattern_init_inches  - 20), -Math.PI / 8)
                            .splineTo(new Vector2d(X_PPG_PATTERN_INIT_INCHES, y_pattern_init_inches ), angle_pattern_init_radians )
                            .build());
        }

        telemetry.addLine("==> INTAKE");
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().addLine("==> INTAKE");
        FtcDashboard.getInstance().getTelemetry().update();


        //
        // mCollecting.startIntake();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .waitSeconds(2)
                        .lineToYConstantHeading(mDrive.getPose().position.y + 30)
                        .build());

        //mCollecting.stopIntake();


        telemetry.addLine("==> GO TO CALIBRATION");
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().addLine("==> GO TO CALIBRATION");
        FtcDashboard.getInstance().getTelemetry().update();

        Actions.runBlocking(
            mDrive.actionBuilder(mDrive.getPose())
                    .waitSeconds(2)
                    //.lineToYConstantHeading(mDrive.getPose().position.y - 50)
                    .splineTo(new Vector2d(X_CALIBRATION_INIT_INCHES ,y_calibration_init_inches ), angle_calibration_init_radians )
                    .build());

        updatePoseFromAprilTagIfVisible();

        telemetry.addLine("===== CALIBRATION =====");
        telemetry.addLine("==> REF POSE : " + mDrive.getPose());
        telemetry.addLine("==> REF OFFSETS X= " + mXOffset + ", Y= " + mYOffset + ", ANG= " + mAngleOffset);
        //telemetry.update();
        FtcDashboard.getInstance().getTelemetry().addLine("===== CALIBRATION =====");
        FtcDashboard.getInstance().getTelemetry().addLine("==> REF POSE : " + mDrive.getPose());
        FtcDashboard.getInstance().getTelemetry().addLine("==> REF OFFSETS X= " + mXOffset + ", Y= " + mYOffset + ", ANG= " + mAngleOffset);
        //FtcDashboard.getInstance().getTelemetry().update();

        telemetry.addLine("======= ACTIONS =======");
        telemetry.addLine("==> GO TO SHOOTING");
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().addLine("======= ACTIONS =======");
        FtcDashboard.getInstance().getTelemetry().addLine("==> GO TO SHOOTING");
        FtcDashboard.getInstance().getTelemetry().update();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .splineTo(new Vector2d(mXOffset + Configuration.X_SHOOTING_FTC_INCHES, mYOffset + Configuration.Y_SHOOTING_FTC_INCHES), mAngleOffset + Configuration.ANGLE_SHOOTING_FTC_RADIANS)
                        .build());

        Configuration.s_Current.persist("Heading",mDrive.getPose().heading.toDouble()- Math.PI /2 );
        Configuration.s_Current.persist("Alliance",mAlliance.getValue());

        mVision.close();


    }

    void updatePoseFromAprilTagIfVisible() {
        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            mReferencePose = new Pose2d(
                    -output.getPosition().x * Configuration.CM_TO_INCHES,
                    -output.getPosition().y * Configuration.CM_TO_INCHES,
                    (output.getOrientation().getYaw() + 180) * Math.PI / 180);

            mDrive.updatePose(mReferencePose);
            mXOffset = 0;
            mYOffset = 0;
            mAngleOffset = 0;

            telemetry.addLine("==> NEW POSE : " + mReferencePose);
            FtcDashboard.getInstance().getTelemetry().addLine("==> NEW POSE : " + mReferencePose);


        }
        else {
            mReferencePose = new Pose2d(0,0,0);
            mXOffset = - X_INIT_FTC_INCHES;
            mYOffset = - Y_INIT_FTC_INCHES;
            mAngleOffset = - ANGLE_INIT_FTC_RADIANS;
        }
    }
}


