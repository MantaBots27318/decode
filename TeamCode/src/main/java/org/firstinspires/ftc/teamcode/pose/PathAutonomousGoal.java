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

    public static final double X_START_INCHES =                         51.6;
    public static final double Y_START_INCHES_BLUE =                    59;
    public static final double Y_START_INCHES_RED =                     - 59;
    public static final double ANGLE_START_RADIANS_RED =                - Math.PI / 180 * 51;
    public static final double ANGLE_START_RADIANS_BLUE =               Math.PI / 180 * 51;

    public static final double X_GPP_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 25 + 8;
    public static final double X_PGP_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 50 + 3;
    public static final double X_PPG_PATTERN_INCHES_BLUE =              -FIELD_SIZE_INCHES / 2 + 9 + 72 + 8;
    public static final double X_GPP_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 30 + 8;
    public static final double X_PGP_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 52 + 5;
    public static final double X_PPG_PATTERN_INCHES_RED =               -FIELD_SIZE_INCHES / 2 + 9 + 75 + 8;
    public static final double Y_PATTERN_INCHES_BLUE =                  36;
    public static final double Y_PATTERN_INCHES_RED =                   -36;
    public static final double ANGLE_PATTERN_RADIANS_BLUE =             Math.PI / 2;
    public static final double ANGLE_PATTERN_RADIANS_RED =              -Math.PI / 2;

    public static final double TGT_INTAKE_TO_CALIBRATION_RADIANS_BLUE = -Math.PI/2;
    public static final double TGT_INTAKE_TO_CALIBRATION_RADIANS_RED =  Math.PI/2;

    public static final double X_CALIBRATION_INCHES =                   11;
    public static final double Y_CALIBRATION_INCHES_BLUE =              20;
    public static final double Y_CALIBRATION_INCHES_RED =               -20;
    public static final double ANGLE_CALIBRATION_RADIANS_RED =          -Math.PI / 4;
    public static final double ANGLE_CALIBRATION_RADIANS_BLUE =         Math.PI / 4;

    public static final double ANGLE_OBELISK_RADIANS_BLUE =             -Math.PI/180 * 60;
    public static final double ANGLE_OBELISK_RADIANS_RED =              Math.PI/180 * 60;

    public static final double X_CALIBRATION_FROM_GOAL_INCHES =         X_START_INCHES - 27;



    Pose2d          mStart                          = new Pose2d(0,0,0);
    Pose2d          mPattern                        = new Pose2d(0,0,0);
    Pose2d          mEndIntake                      = new Pose2d(0,0,0);
    Pose2d          mBackIntake                     = new Pose2d(0,0,0);
    Pose2d          mCalibration                    = new Pose2d(0,0,0);

    double          mTgtIntakeToCalibrationRadians  = 0;
    double          mCalibrationFromGoalInches      = 0;
    double          mAngleObeliskRadians            = 0;

    public PathAutonomousGoal(Logger logger) {
        super(logger);
    }

    public void initialize(Alliance alliance, Pattern pattern, boolean ShallParkInLaunchZone) {

        super.initialize(alliance,ShallParkInLaunchZone);

        if (alliance == Alliance.RED) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_RED, ANGLE_START_RADIANS_RED);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(X_GPP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(X_PGP_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(X_PPG_PATTERN_INCHES_RED,Y_PATTERN_INCHES_RED,ANGLE_PATTERN_RADIANS_RED);
            }

            mEndIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_RED,
                    mPattern.heading.toDouble());
            mCalibration = new Pose2d(X_CALIBRATION_INCHES,Y_CALIBRATION_INCHES_RED,ANGLE_CALIBRATION_RADIANS_RED);

            mTgtIntakeToCalibrationRadians  = TGT_INTAKE_TO_CALIBRATION_RADIANS_RED;
            mAngleObeliskRadians            = ANGLE_OBELISK_RADIANS_RED ;
            mCalibrationFromGoalInches      = X_CALIBRATION_FROM_GOAL_INCHES ;

        }

        if (alliance == Alliance.BLUE) {

            mStart = new Pose2d(X_START_INCHES, Y_START_INCHES_BLUE, ANGLE_START_RADIANS_BLUE);

            if (pattern == Pattern.GPP) {
                mPattern = new Pose2d(X_GPP_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PGP) {
                mPattern = new Pose2d(X_PGP_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }
            if (pattern == Pattern.PPG) {
                mPattern = new Pose2d(X_PPG_PATTERN_INCHES_BLUE, Y_PATTERN_INCHES_BLUE,ANGLE_PATTERN_RADIANS_BLUE);
            }

            mEndIntake = new Pose2d(
                   mPattern.position.x,
                    mPattern.position.y + Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mBackIntake = new Pose2d(
                    mPattern.position.x,
                    mPattern.position.y + 0.4 * Y_DELTA_INTAKE_INCHES_BLUE,
                    mPattern.heading.toDouble());

            mCalibration = new Pose2d(X_CALIBRATION_INCHES,Y_CALIBRATION_INCHES_BLUE,ANGLE_CALIBRATION_RADIANS_BLUE);

            mTgtIntakeToCalibrationRadians  = TGT_INTAKE_TO_CALIBRATION_RADIANS_BLUE;
            mAngleObeliskRadians            = ANGLE_OBELISK_RADIANS_BLUE ;
            mCalibrationFromGoalInches      = X_CALIBRATION_FROM_GOAL_INCHES ;

        }
    }

    public Pose2d   start()                         { return mStart; }
    public Pose2d   pattern()                       { return mPattern; }
    public Pose2d   endIntake()                     { return mEndIntake; }
    public Pose2d   backIntake()                    { return mBackIntake; }
    public Pose2d   calibration()                   { return mCalibration; }

    public double   tgtIntakeToCalibrationRadians() { return mTgtIntakeToCalibrationRadians;}
    public double   xCalibrationFromGoal ()         { return mCalibrationFromGoalInches ;}
    public double   hObeliskFTCRadians ()           { return mAngleObeliskRadians ;}

    public void log() {

        mLogger.info("START X : " + mStart.position.x + " Y: " + mStart.position.y + " H: " + mStart.heading.toDouble());
        mLogger.info("BACKWARDS Y: " + mCalibrationFromGoalInches);
        mLogger.info("PATTERN X : " + mPattern.position.x + " Y: " + mPattern.position.y + " H: " + mPattern.heading.toDouble());
        mLogger.info("END INTAKE X : " + mEndIntake.position.x + " Y: " + mEndIntake.position.y + " H: " + mEndIntake.heading.toDouble());
        mLogger.info("BACK INTAKE X : " + mBackIntake.position.x + " Y: " + mBackIntake.position.y + " H: " + mBackIntake.heading.toDouble());
        mLogger.info("TGT INTAKE TO CALIBRATION INIT : " + mTgtIntakeToCalibrationRadians);
        mLogger.info("CALIBRATION INIT X: " + mCalibration.position.x + " Y: " + mCalibration.position.y + " H: " + mCalibration.heading.toDouble());
        mLogger.info("OBELISK H: " + mAngleObeliskRadians);
        super.log();

    }

}
