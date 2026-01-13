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

    private static final double X_SHOOTING_VERY_FAR_INCHES                      = -52;
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

    private static final double X_QRCODE_INCHES                                  = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_QRCODE_INCHES_BLUE                             = FIELD_SIZE_INCHES / 2 - 9;
    private static final double Y_QRCODE_INCHES_RED                              = - FIELD_SIZE_INCHES / 2 + 9;
    private static final double ANGLE_QRCODE_RADIANS_BLUE                        = 45 * Math.PI / 180;
    private static final double ANGLE_QRCODE_RADIANS_RED                         = -45 * Math.PI / 180;


    Logger          mLogger;

    double          mYDeltaIntakeInches         = 0;

    Pose2d          mShootingClose              = new Pose2d(0,0,0);
    Pose2d          mShootingFar                = new Pose2d(0,0,0);
    Pose2d          mShootingVeryFar            = new Pose2d(0,0,0);
    Pose2d          mParking                    = new Pose2d(0,0,0);
    Pose2d          mQRCode                     = new Pose2d(0,0,0);
    Pose2d          mReady                      = new Pose2d(0,0,0);

    double          mAngleAutoToTeleopRadians   = 0;
    double          mFieldCentric2FTC = 0;

    public Path(Logger logger) {
        mLogger = logger;
    }

    public void initialize(Alliance alliance) {

        if (alliance == Alliance.RED) {


            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_RED,ANGLE_SHOOTING_FAR_RADIANS_RED);
            mShootingVeryFar = new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_RED,ANGLE_SHOOTING_VERY_FAR_RADIANS_RED);
            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_RED,ANGLE_SHOOTING_CLOSE_RADIANS_RED);

            mParking = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_RED,ANGLE_PARKING_LAUNCH_ZONE_RADIANS_RED);
            mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_RED;

            mFieldCentric2FTC = - Math.PI /2;
            mQRCode = new Pose2d(X_QRCODE_INCHES,Y_QRCODE_INCHES_RED,ANGLE_QRCODE_RADIANS_RED);
            
            mReady = new Pose2d(X_READY_INCHES,Y_READY_INCHES_RED,ANGLE_READY_RADIANS_RED);

        }

        if (alliance == Alliance.BLUE) {


            mShootingClose = new Pose2d(X_SHOOTING_CLOSE_INCHES,Y_SHOOTING_CLOSE_INCHES_BLUE,ANGLE_SHOOTING_CLOSE_RADIANS_BLUE);
            mShootingFar = new Pose2d(X_SHOOTING_FAR_INCHES,Y_SHOOTING_FAR_INCHES_BLUE,ANGLE_SHOOTING_FAR_RADIANS_BLUE);
            mShootingVeryFar = new Pose2d(X_SHOOTING_VERY_FAR_INCHES,Y_SHOOTING_VERY_FAR_INCHES_BLUE,ANGLE_SHOOTING_VERY_FAR_RADIANS_BLUE);

            mParking = new Pose2d(X_PARKING_LAUNCH_ZONE_INCHES, Y_PARKING_LAUNCH_ZONE_INCHES_BLUE,ANGLE_PARKING_LAUNCH_ZONE_RADIANS_BLUE);
            mAngleAutoToTeleopRadians = ANGLE_AUTO_TO_TELEOP_LAUNCH_ZONE_RADIANS_BLUE;

            mFieldCentric2FTC = Math.PI /2;
            mQRCode = new Pose2d(X_QRCODE_INCHES,Y_QRCODE_INCHES_BLUE,ANGLE_QRCODE_RADIANS_BLUE);

            mReady = new Pose2d(X_READY_INCHES,Y_READY_INCHES_BLUE,ANGLE_READY_RADIANS_BLUE);

        }
    }

    public Pose2d   shootingClose()         { return mShootingClose; }
    public Pose2d   shootingFar()           { return mShootingFar; }
    public Pose2d   shootingVeryFar()       { return mShootingVeryFar; }
    public Pose2d   parking()               { return mParking;}
    public Pose2d   qrcode()                { return mQRCode; }
    public Pose2d   ready()                 { return mReady; }

    public double   fieldCentric2FTC()      { return mFieldCentric2FTC; }
    public double   hAutoToTeleopRadians()  { return mAngleAutoToTeleopRadians; }

    public void log() {
        mLogger.info("SHOOTING FAR: " + mShootingFar.position.x + " Y: " + mShootingFar.position.y + " H: " + mShootingFar.heading.toDouble());
        mLogger.info("SHOOTING CLOSE: " + mShootingClose.position.x + " Y: " + mShootingClose.position.y + " H: " + mShootingClose.heading.toDouble());
        mLogger.info("LEAVE: " + mParking.position.x + " Y: " + mParking.position.y + " H: " + mParking.heading.toDouble());
        mLogger.info("AUTO TO TELEOP: " + mAngleAutoToTeleopRadians);
        mLogger.info("QRCODE X: " + mQRCode.position.x + " Y: " + mQRCode.position.y + " H: " + mQRCode.heading.toDouble());
    }

}