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

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class Path {



    public static final double FIELD_SIZE_INCHES                                 = 12 * 12;
    public static final double M_TO_INCHES                                       = 39.37;

    private static final double X_PARK_INCHES_RED                                = -39;
    private static final double X_PARK_INCHES_BLUE                               = -35;
    private static final double Y_PARK_INCHES_BLUE                               = -24;
    private static final double Y_PARK_INCHES_RED                                = 24;
    private static final double ANGLE_PARK_RADIANS_BLUE                          = Math.PI / 2;
    private static final double ANGLE_PARK_RADIANS_RED                           = - Math.PI / 2;

    private static final double X_SHOOTING_CLOSE_INCHES                          = 36;
    private static final double Y_SHOOTING_CLOSE_INCHES_BLUE                     = 36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_BLUE                = Math.PI / 4;
    private static final double Y_SHOOTING_CLOSE_INCHES_RED                      = -36;
    private static final double ANGLE_SHOOTING_CLOSE_RADIANS_RED                 = - Math.PI / 4;

    private static final double X_SHOOTING_FAR_INCHES                            = 24;
    private static final double Y_SHOOTING_FAR_INCHES_BLUE                       = 24;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_BLUE                  = Math.PI / 4;
    private static final double Y_SHOOTING_FAR_INCHES_RED                        = -24;
    private static final double ANGLE_SHOOTING_FAR_RADIANS_RED                   = - Math.PI / 4;

    private static final double X_SHOOTING_VERY_FAR_INCHES                      = -52 - 5;
    private static final double Y_SHOOTING_VERY_FAR_INCHES_BLUE                 = 19;
    private static final double ANGLE_SHOOTING_VERY_FAR_RADIANS_BLUE            = 0.365424564;
    private static final double Y_SHOOTING_VERY_FAR_INCHES_RED                  = -19;
    private static final double ANGLE_SHOOTING_VERY_FAR_RADIANS_RED             = -0.365424564;

    private static final double X_PARKING_LAUNCH_ZONE_INCHES                     = 48;
    private static final double Y_PARKING_LAUNCH_ZONE_INCHES_BLUE                = 24;
    private static final double Y_PARKING_LAUNCH_ZONE_INCHES_RED                 = -24;
    private static final double ANGLE_PARKING_LAUNCH_ZONE_RADIANS_RED            = - Math.PI / 4;
    private static final double ANGLE_PARKING_LAUNCH_ZONE_RADIANS_BLUE           = Math.PI / 4;

    private static final double X_READY_INCHES                     = -FIELD_SIZE_INCHES / 2 + 9 + 72 + 7;
    private static final double Y_READY_INCHES_BLUE                = 40;
    private static final double Y_READY_INCHES_RED                 = -40;
    private static final double ANGLE_READY_RADIANS_RED            = - Math.PI / 6;
    private static final double ANGLE_READY_RADIANS_BLUE           = Math.PI / 6;
    
    

    private static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED     = - Math.PI/2;
    private static final double ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE    = Math.PI/2;

    private static final double X_TARGET_INCHES                                  = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_TARGET_INCHES_BLUE                             = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_TARGET_INCHES_RED                              = - FIELD_SIZE_INCHES / 2 + 9;
    private static final double ANGLE_TARGET_RADIANS_BLUE                        = 45 * Math.PI / 180;
    private static final double ANGLE_TARGET_RADIANS_RED                         = -45 * Math.PI / 180;


    Logger          mLogger;

    double          mYDeltaIntakeInches         = 0;

    Pose2d          mShootingClose              = new Pose2d(0,0,0);
    Pose2d          mShootingFar                = new Pose2d(0,0,0);
    Pose2d          mShootingVeryFar            = new Pose2d(0,0,0);
    Pose2d          mLeave                      = new Pose2d(0,0,0);
    Pose2d          mTarget                     = new Pose2d(0,0,0);
    Pose2d          mReady                      = new Pose2d(0,0,0);
    Pose2d          mPark                       = new Pose2d(0,0,0);


    public Path(Logger logger) {
        mLogger = logger;
    }

    public void initialize(Alliance alliance) {

        if (alliance == Alliance.RED) {
            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_RED,ANGLE_SHOOTING_FAR_RADIANS_RED);
            mShootingVeryFar = new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_RED,ANGLE_SHOOTING_VERY_FAR_RADIANS_RED);
            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_RED,ANGLE_SHOOTING_CLOSE_RADIANS_RED);

            mTarget = new Pose2d(X_TARGET_INCHES,Y_TARGET_INCHES_RED,ANGLE_TARGET_RADIANS_RED);
            
            mReady = new Pose2d(X_READY_INCHES,Y_READY_INCHES_RED,ANGLE_READY_RADIANS_RED);
            mLeave = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_RED,ANGLE_PARKING_LAUNCH_ZONE_RADIANS_RED);
            mPark = new Pose2d(X_PARK_INCHES_RED,Y_PARK_INCHES_RED,ANGLE_PARK_RADIANS_RED);
        }

        if (alliance == Alliance.BLUE) {
            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_BLUE,ANGLE_SHOOTING_CLOSE_RADIANS_BLUE);
            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_BLUE,ANGLE_SHOOTING_FAR_RADIANS_BLUE);
            mShootingVeryFar = new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_BLUE,ANGLE_SHOOTING_VERY_FAR_RADIANS_BLUE);

            mTarget = new Pose2d(X_TARGET_INCHES,Y_TARGET_INCHES_BLUE,ANGLE_TARGET_RADIANS_BLUE);

            mReady = new Pose2d(X_READY_INCHES,Y_READY_INCHES_BLUE,ANGLE_READY_RADIANS_BLUE);
            mPark = new Pose2d(X_PARK_INCHES_BLUE,Y_PARK_INCHES_BLUE,ANGLE_PARK_RADIANS_BLUE);
            mLeave = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_BLUE,ANGLE_PARKING_LAUNCH_ZONE_RADIANS_BLUE);
        }
    }

    public Pose2d   shootingClose()         { return mShootingClose; }
    public Pose2d   shootingFar()           { return mShootingFar; }
    public Pose2d   shootingVeryFar()       { return mShootingVeryFar; }

    public Pose2d   target()                { return mTarget; }
    public Pose2d   leave()                 { return mLeave;}
    public Pose2d   ready()                 { return mReady; }

    public void log() {
        mLogger.info("SHOOTING VERY FAR: " + mShootingVeryFar.position.x + " Y: " + mShootingVeryFar.position.y + " H: " + mShootingVeryFar.heading.toDouble());
        mLogger.info("SHOOTING FAR: " + mShootingFar.position.x + " Y: " + mShootingFar.position.y + " H: " + mShootingFar.heading.toDouble());
        mLogger.info("SHOOTING CLOSE: " + mShootingClose.position.x + " Y: " + mShootingClose.position.y + " H: " + mShootingClose.heading.toDouble());
        mLogger.info("LEAVE: " + mLeave.position.x + " Y: " + mLeave.position.y + " H: " + mLeave.heading.toDouble());
        mLogger.info("READY: " + mReady.position.x + " Y: " + mReady.position.y + " H: " + mReady.heading.toDouble());
        mLogger.info("TARGET X: " + mTarget.position.x + " Y: " + mTarget.position.y + " H: " + mTarget.heading.toDouble());
    }

}