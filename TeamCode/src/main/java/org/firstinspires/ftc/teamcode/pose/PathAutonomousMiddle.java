/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing path reference positions for autonomous
   middle
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* ACME includes */
import com.acmerobotics.roadrunner.Pose2d;

/* Project includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Pattern;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

public class PathAutonomousMiddle extends Path {

    private static final double X_START_INCHES                               = -53.5;
    private static final double Y_START_INCHES_BLUE                          = 18;
    private static final double Y_START_INCHES_RED                           = -18;
    private static final double ANGLE_START_RADIANS                          = 0;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE                 = 41;
    protected static final double Y_DELTA_INTAKE_INCHES_RED                  = -41;


    private static final double X_DELTA_GPP_PATTERN_INCHES_BLUE              = 22;
    private static final double X_DELTA_PGP_PATTERN_INCHES_BLUE              = 45;
    private static final double X_DELTA_PPG_PATTERN_INCHES_BLUE              = 71;
    private static final double X_DELTA_GPP_PATTERN_INCHES_RED               = 24;
    private static final double X_DELTA_PGP_PATTERN_INCHES_RED               = 45;
    private static final double X_DELTA_PPG_PATTERN_INCHES_RED               = 71;

    private static final double X_DELTA_PGP_PATTERN_INCHES_NEXT_BLUE         = 45;
    private static final double X_DELTA_PPG_PATTERN_INCHES_NEXT_BLUE         = 70;
    private static final double X_DELTA_PGP_PATTERN_INCHES_NEXT_RED          = 46;
    private static final double X_DELTA_PPG_PATTERN_INCHES_NEXT_RED          = 71;
    private static final double Y_DELTA_PATTERN_INCHES_BLUE                  = 10;
    private static final double Y_DELTA_PATTERN_INCHES_RED                   = -10;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_BLUE             = Math.PI / 2;
    private static final double ANGLE_DELTA_PATTERN_RADIANS_RED              = -Math.PI / 2;

    private static final double TGT_DELTA_INTAKE_TO_SHOOT_RADIANS_BLUE = -Math.PI/2;
    private static final double TGT_DELTA_INTAKE_TO_SHOOT_RADIANS_RED  = Math.PI/2;

    private static final double X_LEAVE_VERY_FAR_INCHES                      = -52;
    private static final double Y_LEAVE_VERY_FAR_INCHES_BLUE                 = 39;
    private static final double ANGLE_LEAVE_VERY_FAR_RADIANS_BLUE            = 0;
    private static final double Y_LEAVE_VERY_FAR_INCHES_RED                  = -39;
    private static final double ANGLE_LEAVE_VERY_FAR_RADIANS_RED             = -0;

    private static final double X_ZELIE_INCHES                      = -52;
    private static final double Y_ZELIE_INCHES_BLUE                 = 19;
    private static final double ANGLE_ZELIE_RADIANS_BLUE            = 0.365424564;
    private static final double Y_ZELIE_INCHES_RED                  = -19;
    private static final double ANGLE_ZELIE_RADIANS_RED             = -0.365424564;


    Pose2d  mStart                          = new Pose2d(0,0,0);
    Pose2d  mPattern                        = new Pose2d(0,0,0);
    Pose2d  mEndIntake                      = new Pose2d(0,0,0);
    Pose2d  mBackIntake                     = new Pose2d(0,0,0);
    Pose2d  mNextPattern                    = new Pose2d(0,0,0);
    Pose2d  mEndNextIntake                  = new Pose2d(0,0,0);
    Pose2d  mBackNextIntake                 = new Pose2d(0,0,0);
    Pose2d  mLeaveVeryFar                 = new Pose2d(0,0,0);
    Pose2d  mZelie                 = new Pose2d(0,0,0);
    double  mTgtIntakeToShootRadians  = 0;

    public PathAutonomousMiddle(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance, Pattern pattern) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_RED;


            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_NEXT_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_NEXT_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_NEXT_RED,
                        Y_START_INCHES_RED + Y_DELTA_PATTERN_INCHES_RED,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_RED);
            }

            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.3 * Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mEndNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mNextPattern.heading.toDouble());

            mBackNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + 0.7 * Y_DELTA_INTAKE_INCHES_RED,
                    0);

            mTgtIntakeToShootRadians = TGT_DELTA_INTAKE_TO_SHOOT_RADIANS_RED + ANGLE_START_RADIANS;
            mLeaveVeryFar = new Pose2d(X_LEAVE_VERY_FAR_INCHES,Y_LEAVE_VERY_FAR_INCHES_RED,ANGLE_LEAVE_VERY_FAR_RADIANS_RED);
            mZelie = new Pose2d(X_ZELIE_INCHES,Y_ZELIE_INCHES_RED,ANGLE_ZELIE_RADIANS_RED);


        }
        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE,ANGLE_START_RADIANS);

            mYDeltaIntakeInches = Y_DELTA_INTAKE_INCHES_BLUE;

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_GPP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_NEXT_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_NEXT_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PPG_PATTERN_INCHES_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(
                        X_START_INCHES + X_DELTA_PGP_PATTERN_INCHES_NEXT_BLUE,
                        Y_START_INCHES_BLUE + Y_DELTA_PATTERN_INCHES_BLUE,
                        ANGLE_START_RADIANS + ANGLE_DELTA_PATTERN_RADIANS_BLUE);
            }

            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.3 * Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());
            
            mEndNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mNextPattern.heading.toDouble());

            mBackNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + 0.7 * Y_DELTA_INTAKE_INCHES_BLUE,
                    0);

           mTgtIntakeToShootRadians = TGT_DELTA_INTAKE_TO_SHOOT_RADIANS_BLUE + ANGLE_START_RADIANS;
            mLeaveVeryFar = new Pose2d(X_LEAVE_VERY_FAR_INCHES,Y_LEAVE_VERY_FAR_INCHES_BLUE,ANGLE_LEAVE_VERY_FAR_RADIANS_BLUE);
            mZelie = new Pose2d(X_ZELIE_INCHES,Y_ZELIE_INCHES_BLUE,ANGLE_ZELIE_RADIANS_BLUE);

        }
    }

    public Pose2d   start()                         { return mStart; }
    public Pose2d   pattern()                       { return mPattern; }
    public Pose2d   endIntake()                     { return mEndIntake; }
    public Pose2d   backIntake()                    { return mBackIntake; }
    public Pose2d   nextPattern()                   { return mNextPattern; }
    public Pose2d   endNextIntake()                 { return mEndNextIntake; }
    public Pose2d   backNextIntake()                { return mBackNextIntake; }
    public Pose2d   leaveVeryFar()                 { return mLeaveVeryFar; }
    public Pose2d   zelie()                { return mZelie; }

    public double   tgtIntakeToShootRadians() { return mTgtIntakeToShootRadians;}

    public void log() {

        mLogger.info("START X : " + mStart.position.x + " Y: " + mStart.position.y + " H: " + mStart.heading.toDouble());
        mLogger.info("PATTERN X : " + mPattern.position.x + " Y: " + mPattern.position.y + " H: " + mPattern.heading.toDouble());
        mLogger.info("END INTAKE X : " + mEndIntake.position.x + " Y: " + mEndIntake.position.y + " H: " + mEndIntake.heading.toDouble());
        mLogger.info("BACK INTAKE X : " + mBackIntake.position.x + " Y: " + mBackIntake.position.y + " H: " + mBackIntake.heading.toDouble());
        mLogger.info("TGT INTAKE TO SHOOT INIT : " + mTgtIntakeToShootRadians);
        super.log();
    }

}
