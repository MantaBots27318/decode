package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower createFollower(HardwareMap hardwareMap) {

        PinpointConstants pinpoint = new PinpointConstants()
                .hardwareMapName("pinpoint")
                .forwardPodY(1.6)
                .strafePodX(-(6.0 + 14.0 / 16))
                .distanceUnit(DistanceUnit.INCH)
                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        MecanumConstants mecanum = new MecanumConstants()
                .leftFrontMotorName("front_left_motor")
                .leftRearMotorName("back_left_motor")
                .rightFrontMotorName("front_right_motor")
                .rightRearMotorName("back_right_motor")
                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

        return new FollowerBuilder(new FollowerConstants(), hardwareMap)
                .pinpointLocalizer(pinpoint)
                .mecanumDrivetrain(mecanum)
                .build();
    }
}