/* -------------------------------------------------------
   Copyright (c) [2025] FASNY
   All rights reserved
   -------------------------------------------------------
   MotorMock Enables mocking of a motor for testing purpose
   ------------------------------------------------------- */
package org.firstinspires.ftc.teamcode.components;

/* Qualcomm includes */
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class MotorMock extends MotorComponent {

    DcMotorSimple.Direction     mDirection;
    DcMotor.RunMode             mMode;
    int                         mPosition;
    DcMotor.ZeroPowerBehavior   mBehavior;
    double                      mPower;
    int                         mTolerance;

    public MotorMock(String name)
    {
        mName = name;
        mDirection = DcMotorSimple.Direction.FORWARD;
        mMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        mBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;
    }

    @Override
    public int	                        getCurrentPosition() { return mPosition; }

    @Override
    public DcMotorSimple.Direction      getDirection()
    {
        return mDirection;
    }

    @Override
    public DcMotor.RunMode	            getMode() { return mMode; }

    @Override
    public int	                        getTargetPosition() { return mPosition; }

    @Override
    public DcMotor.ZeroPowerBehavior	getZeroPowerBehavior() { return mBehavior; }

    @Override
    public String                       logPositions() { return "" + this.getCurrentPosition(); }

    @Override
    public double                   	getPower() { return mPower; }

    @Override
    public boolean	                    isBusy() { return false; }

    @Override
    public void	                        setMode(DcMotor.RunMode mode) { mMode = mode; }

    @Override
    public void	                        setDirection(DcMotorSimple.Direction direction) { mDirection = direction; }

    @Override
    public void	                        setTargetPosition(int position) { mPosition = position;}

    @Override
    public void	                        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) { mBehavior = zeroPowerBehavior; }

    @Override
    public void	                        setPower(double power) { mPower = power; }

    @Override
    public PIDFCoefficients             getPIDFCoefficients(DcMotor.RunMode mode) { return null; }

    @Override
    public void                         setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients){
    }

    @Override
    public void                         setTargetPositionTolerance(int tolerance)
    {
        mTolerance = tolerance;
    }

    @Override
    public int                          getTargetPositionTolerance()
    {
        return mTolerance;
    }

    @Override
    public double                       getVelocity()
    {
        return 0.0;
    }

    @Override
    public void                         setVelocity(double rate) { }

}
