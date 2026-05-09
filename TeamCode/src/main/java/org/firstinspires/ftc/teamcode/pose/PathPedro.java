/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Generic class managing path reference positions for autonomous
   opmodes
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* ACME ROBOTICS include */

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.configurations.Alliance;
import org.firstinspires.ftc.teamcode.utils.Logger;
import org.firstinspires.ftc.teamcode.utils.PoseConvertion;

public class PathPedro {



    public static final double FIELD_SIZE_INCHES                                 = 12 * 12;
    public static final double M_TO_INCHES                                       = 39.37;

    private static final double X_PARK_INCHES_RED                                = -42;
    private static final double X_PARK_INCHES_BLUE                               = -42;
    private static final double Y_PARK_INCHES_BLUE                               = -34;
    private static final double Y_PARK_INCHES_RED                                = 34;
    private static final double ANGLE_PARK_RADIANS_BLUE                          = Math.PI / 2;
    private static final double ANGLE_PARK_RADIANS_RED                           = - Math.PI / 2;

    private static final double X_SHOOTING_CLOSE_INCHES                          = 36;
    private static final double Y_SHOOTING_CLOSE_INCHES_BLUE                     = 36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_BLUE                = Math.PI / 2;
    private static final double Y_SHOOTING_CLOSE_INCHES_RED                      = -36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_RED                 = - Math.PI / 2;

    private static final double X_SHOOTING_FAR_INCHES                            = 24;
    private static final double Y_SHOOTING_FAR_INCHES_BLUE                       = 24;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_BLUE                  = Math.PI / 2;
    private static final double Y_SHOOTING_FAR_INCHES_RED                        = -24;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_RED                   = - Math.PI / 2;

    private static final double X_SHOOTING_VERY_FAR_INCHES                      = -59;
    private static final double Y_SHOOTING_VERY_FAR_INCHES_BLUE                 = 20;
    private static final double ANGLE_SHOOTING_VERY_FAR_RADIANS_BLUE            = 0;
    private static final double Y_SHOOTING_VERY_FAR_INCHES_RED                  = -20;
    private static final double ANGLE_SHOOTING_VERY_FAR_RADIANS_RED             = 0;

    private static final double X_READY_INCHES                     = -FIELD_SIZE_INCHES / 2 + 9 + 72 + 7;
    private static final double Y_READY_INCHES_BLUE                = 40;
    private static final double Y_READY_INCHES_RED                 = -40;
    private static final double ANGLE_READY_RADIANS_RED            = - Math.PI / 6;
    private static final double ANGLE_READY_RADIANS_BLUE           = Math.PI / 6;


    private static final double X_TARGET_INCHES                                  = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_TARGET_INCHES_BLUE                             = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_TARGET_INCHES_RED                              = - FIELD_SIZE_INCHES / 2 + 9;
    private static final double ANGLE_TARGET_RADIANS_BLUE                        = 45 * Math.PI / 180;
    private static final double ANGLE_TARGET_RADIANS_RED                         = -45 * Math.PI / 180;

    private static final double ANGLE_FC_TO_FTC_BLUE                             = Math.PI / 2;
    private static final double ANGLE_FC_TO_FTC_RED                              = -Math.PI/2;




    Logger          mLogger;

    double          mYDeltaIntakeInches         = 0;
    double          mFC2FTC                     = 0;

    Pose mShootingClose              = new Pose(0,0,0);
    Pose         mShootingFar                = new Pose(0,0,0);
    Pose          mShootingVeryFar            = new Pose(0,0,0);
    Pose         mTarget                     = new Pose(0,0,0);
    Pose          mReady                      = new Pose(0,0,0);
    Pose          mPark                       = new Pose(0,0,0);


    public PathPedro(Logger logger) {
        mLogger = logger;
    }

    public void initialize(Alliance alliance) {

        if (alliance == Alliance.RED) {
            mShootingFar = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_RED,ANGLE_SHOOTING_FAR_RADIANS_RED));
            mShootingVeryFar = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_RED,ANGLE_SHOOTING_VERY_FAR_RADIANS_RED));
            mShootingClose = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_RED,ANGLE_SHOOTING_CLOSE_RADIANS_RED));

            mTarget = PoseConvertion.toPedroPose(new Pose2d(X_TARGET_INCHES,Y_TARGET_INCHES_RED,ANGLE_TARGET_RADIANS_RED));
            
            mReady = PoseConvertion.toPedroPose(new Pose2d(X_READY_INCHES,Y_READY_INCHES_RED,ANGLE_READY_RADIANS_RED));
            mPark = PoseConvertion.toPedroPose(new Pose2d(X_PARK_INCHES_RED,Y_PARK_INCHES_RED,ANGLE_PARK_RADIANS_RED));

            mFC2FTC = ANGLE_FC_TO_FTC_RED;
        }

        if (alliance == Alliance.BLUE) {
            mShootingClose = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_BLUE,ANGLE_SHOOTING_CLOSE_RADIANS_BLUE));
            mShootingFar = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_BLUE,ANGLE_SHOOTING_FAR_RADIANS_BLUE));
            mShootingVeryFar = PoseConvertion.toPedroPose(new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_BLUE,ANGLE_SHOOTING_VERY_FAR_RADIANS_BLUE));

            mTarget = PoseConvertion.toPedroPose(new Pose2d(X_TARGET_INCHES,Y_TARGET_INCHES_BLUE,ANGLE_TARGET_RADIANS_BLUE));

            mReady = PoseConvertion.toPedroPose(new Pose2d(X_READY_INCHES,Y_READY_INCHES_BLUE,ANGLE_READY_RADIANS_BLUE));
            mPark = PoseConvertion.toPedroPose(new Pose2d(X_PARK_INCHES_BLUE,Y_PARK_INCHES_BLUE,ANGLE_PARK_RADIANS_BLUE));

            mFC2FTC = ANGLE_FC_TO_FTC_BLUE;
        }
    }

    public Pose   shootingClose()         { return mShootingClose; }
    public Pose   shootingFar()           { return mShootingFar; }
    public Pose   shootingVeryFar()       { return mShootingVeryFar; }

    public Pose   target()                { return mTarget; }
    public Pose   park()                  { return mPark; }

    public double   FC2FTC()                { return mFC2FTC; }

    public void log() {
        mLogger.info(Logger.Target.DRIVER_STATION,"SHOOTING VERY FAR: " + mShootingVeryFar.getX() + " Y: " + mShootingVeryFar.getY() + " H: " + mShootingVeryFar.getHeading());
        mLogger.info(Logger.Target.DRIVER_STATION,"SHOOTING FAR: " + mShootingFar.getX() + " Y: " + mShootingFar.getY() + " H: " + mShootingFar.getHeading());
        mLogger.info(Logger.Target.DRIVER_STATION,"SHOOTING CLOSE: " + mShootingClose.getX() + " Y: " + mShootingClose.getY() + " H: " + mShootingClose.getHeading());
        mLogger.info(Logger.Target.DRIVER_STATION,"READY: " + mReady.getX() + " Y: " + mReady.getY() + " H: " + mReady.getHeading());
        mLogger.info(Logger.Target.DRIVER_STATION,"TARGET X: " + mTarget.getX() + " Y: " + mTarget.getY() + " H: " + mTarget.getHeading());
    }

}