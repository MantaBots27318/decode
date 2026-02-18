/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   Class managing path reference positions for autonomous
   starting at the goal
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.pose;

/* ACME robotics includes /*/
import com.acmerobotics.roadrunner.Pose2d;

/* Configuration includes */
import org.firstinspires.ftc.teamcode.configurations.Alliance;

/* Utils includes */
import org.firstinspires.ftc.teamcode.utils.Logger;

/* Vision includes */
import org.firstinspires.ftc.teamcode.vision.Pattern;

public class PathAutonomousGoal extends Path {

    public static final double X_START_INCHES =                         53.5;
    public static final double Y_START_INCHES_BLUE =                    61;
    public static final double Y_START_INCHES_RED =                     - 61;
    public static final double ANGLE_START_RADIANS_RED =                - Math.PI / 180 * 51;
    public static final double ANGLE_START_RADIANS_BLUE =               Math.PI / 180 * 51;


    protected static final double Y_DELTA_INTAKE_INCHES_BLUE             = 32;
    protected static final double Y_DELTA_INTAKE_INCHES_RED             = -32;

    public static final double X_GPP_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 25 + 7;
    public static final double X_PGP_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 48;
    public static final double X_PPG_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 72 + 3;
    public static final double X_GPP_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 30 + 3;
    public static final double X_PGP_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 52 + 5;
    public static final double X_PPG_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 75 + 8;


    public static final double Y_PATTERN_INCHES_BLUE =                  35;
    public static final double Y_PATTERN_INCHES_RED =                   -35;
    public static final double ANGLE_PATTERN_RADIANS_BLUE =             Math.PI / 2;
    public static final double ANGLE_PATTERN_RADIANS_RED =              -Math.PI / 2;

    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_BLUE        = -Math.PI/2;
    public static final double TGT_INTAKE_TO_SHOOT_RADIANS_RED         =  Math.PI/2;


    public static final double ANGLE_OBELISK_RADIANS_BLUE =             -Math.PI/180 * 80;
    public static final double ANGLE_OBELISK_RADIANS_RED =              Math.PI/180 * 80;

    public static final double X_SHOOT_FROM_GOAL_INCHES =         X_START_INCHES - 27;



    Pose2d          mStart                          = new Pose2d(0,0,0);
    Pose2d          mPattern                        = new Pose2d(0,0,0);
    Pose2d          mNextPattern                    = new Pose2d(0,0,0);
    Pose2d          mEndIntake                      = new Pose2d(0,0,0);
    Pose2d          mBackIntake                     = new Pose2d(0,0,0);
    Pose2d          mEndNextIntake                  = new Pose2d(0,0,0);
    Pose2d          mBackNextIntake                 = new Pose2d(0,0,0);

    double          mTgtIntakeToShootRadians  = 0;
    double          mShootFromGoalInches      = 0;
    double          mAngleObeliskRadians      = 0;

    public PathAutonomousGoal(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance, Pattern pattern) {

        super.initialize(alliance);

        if (alliance == Alliance.RED) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS_RED);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(X_GPP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(X_PGP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
                mNextPattern = new Pose2d(X_PGP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }

            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mEndNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mNextPattern.heading.toDouble());

            mBackNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + 0.5 * Y_DELTA_INTAKE_INCHES_RED,
                    mNextPattern.heading.toDouble());

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_RED;
            mAngleObeliskRadians      = ANGLE_OBELISK_RADIANS_RED ;
            mShootFromGoalInches      = X_SHOOT_FROM_GOAL_INCHES ;

        }

        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(X_GPP_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(X_PPG_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(X_PGP_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(X_PPG_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(X_PPG_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
                mNextPattern = new Pose2d(X_PGP_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }

            mEndIntake = new Pose2d(
                   mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mEndNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mNextPattern.heading.toDouble());

            mBackNextIntake = new Pose2d(
                    mNextPattern.position.x,
                    mNextPattern.position.y + 0.5 * Y_DELTA_INTAKE_INCHES_BLUE,
                    mNextPattern.heading.toDouble());

            mTgtIntakeToShootRadians  = TGT_INTAKE_TO_SHOOT_RADIANS_BLUE;
            mAngleObeliskRadians            = ANGLE_OBELISK_RADIANS_BLUE ;
            mShootFromGoalInches      = X_SHOOT_FROM_GOAL_INCHES ;

        }
    }

    public Pose2d   start()                         { return mStart; }
    public Pose2d   pattern()                       { return mPattern; }
    public Pose2d   nextPattern()                   { return mNextPattern; }
    public Pose2d   endIntake()                     { return mEndIntake; }
    public Pose2d   backIntake()                    { return mBackIntake; }
    public Pose2d   endNextIntake()                 { return mEndNextIntake; }
    public Pose2d   backNextIntake()                { return mBackNextIntake; }

    public double   tgtIntakeToShootRadians() { return mTgtIntakeToShootRadians;}
    public double   xShootFromGoal ()         { return mShootFromGoalInches ;}
    public double   hObeliskFTCRadians ()           { return mAngleObeliskRadians ;}

    public void log() {

        mLogger.info("START X : " + mStart.position.x + " Y: " + mStart.position.y + " H: " + mStart.heading.toDouble());
        mLogger.info("BACKWARDS Y: " + mShootFromGoalInches);
        mLogger.info("PATTERN X : " + mPattern.position.x + " Y: " + mPattern.position.y + " H: " + mPattern.heading.toDouble());
        mLogger.info("END INTAKE X : " + mEndIntake.position.x + " Y: " + mEndIntake.position.y + " H: " + mEndIntake.heading.toDouble());
        mLogger.info("BACK INTAKE X : " + mBackIntake.position.x + " Y: " + mBackIntake.position.y + " H: " + mBackIntake.heading.toDouble());
        mLogger.info("TGT INTAKE TO SHOOT INIT : " + mTgtIntakeToShootRadians);
        mLogger.info("OBELISK H: " + mAngleObeliskRadians);
        super.log();

    }

}
