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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
import org.firstinspires.ftc.teamcode.configurations.Poses;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.SmartTimer;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.components.Controller;
import org.firstinspires.ftc.teamcode.camera.Camera;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutonomousGoalStart extends LinearOpMode {

    Vision          mVision;
    MecanumDrive    mDrive;
    Collecting      mCollecting;

    Vision.Pattern  mPattern;
    Vision.Pattern  mTargetPattern;
    int             mPatternShift = 0;
    Alliance        mAlliance = Alliance.NONE;
    Poses           mPoses;
    double          mWaitingTime = 0.0;

    Pose2d          mReferencePose;

    SmartTimer      mTimer;
    double          mXOffset        = 0;
    double          mYOffset        = 0;
    double          mAngleOffset    = 0;

    Controller      mGamepad1;
    Controller      mGamepad2;
    Camera          mCamera;

    List mLogs = new ArrayList<String>();

    @Override
    public void runOpMode() throws InterruptedException {

        mCollecting     = new Collecting();

        mTimer          = new SmartTimer(telemetry);

        mCamera         = new Camera();
        mCamera.setHW(Configuration.s_Current,hardwareMap,telemetry);

        mVision         = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();
        mPattern        = Vision.Pattern.NONE;
        mTargetPattern  = Vision.Pattern.NONE;

        mPoses          = new Poses(FtcDashboard.getInstance().getTelemetry());
        mReferencePose  = new Pose2d(0, 0, 0);

        mDrive          = new MecanumDrive(hardwareMap, mReferencePose);

        mGamepad1       = new Controller(gamepad1, telemetry);
        mGamepad2       = new Controller(gamepad2, telemetry);
        boolean mShallParkInLaunchZone = false;

        mCollecting     = new Collecting();
        mCollecting.setHW(Configuration.s_Current, hardwareMap, telemetry, mGamepad2);

        while (opModeInInit()) {

            if (mGamepad1.buttons.dpad_right.pressedOnce())         {
                mAlliance = Alliance.RED;
                mPoses.initialize(mAlliance,mTargetPattern,mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.dpad_left.pressedOnce())          {
                mAlliance = Alliance.BLUE;
                mPoses.initialize(mAlliance, mTargetPattern, mShallParkInLaunchZone);
            }

            if (mGamepad1.buttons.dpad_up.pressedOnce())            { mWaitingTime += 1; }
            if (mGamepad1.buttons.dpad_down.pressedOnce())          { mWaitingTime -= 1; mWaitingTime = Math.max(mWaitingTime,0); }

            if (mGamepad1.buttons.left_stick_x_left.pressedOnce())  {
                mPatternShift -= 1;
                mPatternShift = Math.max(mPatternShift,0);
                mTargetPattern = this.computePattern(mPattern,mPatternShift);
                mPoses.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }
            if (mGamepad1.buttons.left_stick_x_right.pressedOnce()) {
                mPatternShift += 1;
                mPatternShift = Math.min(mPatternShift,3);
                mTargetPattern = this.computePattern(mPattern,mPatternShift);
                mPoses.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }

            Vision.Pattern pattern = mVision.readPattern();
            if (pattern != Vision.Pattern.NONE) {
                mPattern = pattern;
                mTargetPattern = this.computePattern(mPattern,mPatternShift);
                mPoses.initialize(mAlliance, mTargetPattern,mShallParkInLaunchZone);
            }
            // Display menu
            telemetry.addLine("=========== MENU ============");
            telemetry.addLine("Choose Alliance: DPAD LEFT/RIGHT");
            telemetry.addLine("Choose Pattern Shift: LEFT STICK X LEFT/RIGHT");
            telemetry.addLine("Choose Waiting Time: DPAD UP/DOWN");

            telemetry.addLine("======= CONFIGURATION =======");
            telemetry.addData("==> PATTERN : " , mPattern.text());
            telemetry.addData("==> PATTERN SHIFT : " , mPatternShift);
            telemetry.addData("==> PATTERN TARGET : " , mTargetPattern.text());
            telemetry.addData("==> ALLIANCE : ", mAlliance.name());
            telemetry.addData("==> WAITING TIME : ", mWaitingTime + " s");
            telemetry.update();

            FtcDashboard.getInstance().getTelemetry().addLine("======= CONFIGURATION =======");
            FtcDashboard.getInstance().getTelemetry().addData("PATTERN" , mPattern.text());
            FtcDashboard.getInstance().getTelemetry().addData("PATTERN SHIFT" , mPatternShift);
            FtcDashboard.getInstance().getTelemetry().addData("PATTERN TARGET" , mTargetPattern.text());
            FtcDashboard.getInstance().getTelemetry().addData("ALLIANCE" , mAlliance.name());
            FtcDashboard.getInstance().getTelemetry().addData("WAITING TIME" , mWaitingTime + " s");
            mPoses.log();
            FtcDashboard.getInstance().getTelemetry().update();

        }

        mLogs.add("======= ACTIONS =======");
        mLogs.add("==> GO TO SHOOTING POSITION");

        for (Object l : mLogs) { telemetry.addLine(l.toString());}
        telemetry.update();
        for (Object l : mLogs) { FtcDashboard.getInstance().getTelemetry().addLine(l.toString());}
        FtcDashboard.getInstance().getTelemetry().update();

        Actions.runBlocking(
                mDrive.actionBuilder(mReferencePose)
                        .waitSeconds(mWaitingTime)
                        .lineToXConstantHeading(mPoses.xShootingFromGoal())
                        .build());

        updatePoseFromAprilTagIfVisible();

        mLogs.add("==> CALIBRATION");
        mLogs.add("REF POSE :" + mDrive.getPose());
        mLogs.add("REF OFFSETS X= " + mXOffset + ", Y= " + mYOffset + ", ANG= " + mAngleOffset);

        mLogs.add("==> GO TO SHOOTING");

        for (Object l : mLogs) { telemetry.addLine(l.toString());}
        telemetry.update();
        for (Object l : mLogs) { FtcDashboard.getInstance().getTelemetry().addLine(l.toString());}
        FtcDashboard.getInstance().getTelemetry().update();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .waitSeconds(0.5)
                        .splineToLinearHeading(new Pose2d(new Vector2d(mXOffset + mPoses.posShootingFTCInches().x, mYOffset + mPoses.posShootingFTCInches().y), mAngleOffset + mPoses.hShootingFTCRadians()),mAngleOffset + mPoses.hShootingFTCRadians())
                        .build());



        mLogs.add("==> Shoot");
        for (Object l : mLogs) { telemetry.addLine(l.toString());}
        telemetry.update();
        for (Object l : mLogs) { FtcDashboard.getInstance().getTelemetry().addLine(l.toString());}
        FtcDashboard.getInstance().getTelemetry().update();

        mCollecting.shoot() ;


        mLogs.add("==> Leave");
        for (Object l : mLogs) { telemetry.addLine(l.toString());}
        telemetry.update();
        for (Object l : mLogs) { FtcDashboard.getInstance().getTelemetry().addLine(l.toString());}
        FtcDashboard.getInstance().getTelemetry().update();

        Actions.runBlocking(
                mDrive.actionBuilder(mDrive.getPose())
                        .waitSeconds(2)
                        .setTangent(mDrive.getPose().heading.toDouble() + Math.PI )
                        .splineToLinearHeading(new Pose2d(mPoses.posLeaveGoalInches() , mPoses.hLeaveGoalRadians()),0)
                        .build());

//



        Configuration.s_Current.persist("heading",mDrive.getPose().heading.toDouble() + mPoses.hAutoToTeleopRadians() );
        Configuration.s_Current.persist("alliance",mAlliance.getValue());

        mVision.close();

    }

    void updatePoseFromAprilTagIfVisible() {
        // Gather April Tags
        Pose3D output = mVision.getPosition();
        mTimer.arm(100);

        while(output == null && mTimer.isArmed()) { output = mVision.getPosition(); }
        if (output != null) {

            mReferencePose = new Pose2d(
                    -output.getPosition().x * Poses.M_TO_INCHES,
                    -output.getPosition().y * Poses.M_TO_INCHES,
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
            mXOffset = - mPoses.posInitFTCInches().x;
            mYOffset = - mPoses.posInitFTCInches().y;
            mAngleOffset = - mPoses.hInitFTCInches();
        }
    }

    Vision.Pattern  computePattern(Vision.Pattern official, int shift) {

        Vision.Pattern result = Vision.Pattern.NONE;

        if(official != Vision.Pattern.NONE) {
            int target_identifier = (official.identifier() + shift) % 3;
            if (target_identifier == Vision.Pattern.GPP.identifier()) {
                result = Vision.Pattern.GPP;
            }
            if (target_identifier == Vision.Pattern.PGP.identifier()) {
                result = Vision.Pattern.PGP;
            }
            if (target_identifier == Vision.Pattern.PPG.identifier()) {
                result = Vision.Pattern.PPG;
            }
        }

        return result;
    }

}



//
//        mLogs.add( "==> GO TO CALIBRATION");
//        for (Object l : mLogs) { telemetry.addLine(l.toString());}
//        telemetry.update();
//        for (Object l : mLogs) { FtcDashboard.getInstance().getTelemetry().addLine(l.toString());}
//        FtcDashboard.getInstance().getTelemetry().update();

//        Actions.runBlocking(
//                mDrive.actionBuilder(mDrive.getPose())
//                        //.waitSeconds(2)
//                        //.lineToYConstantHeading(mDrive.getPose().position.y - mPoses.yDeltaIntakeInches() * 0.25)
//                        //.splineToLinearHeading(new Pose2d(mPoses.posCalibrationInitInches(), mPoses.hCalibrationInitRadians()),0)
//                        .setTangent(Math.PI / 2)
//                        .splineToLinearHeading(new Pose2d(mPoses.posCalibrationInitInches(), mPoses.hCalibrationInitRadians()),0, new TranslationalVelConstraint(100), new ProfileAccelConstraint(-50,50))
//                        .build());
