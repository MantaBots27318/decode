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




import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.configurations.Configuration;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.Vision;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */

@Autonomous
public class AutonomousMiddleStart extends LinearOpMode {

    Vision mVision;
    MecanumDrive mDrive;
    Collecting mCollecting;

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.setMsTransmissionInterval(11);
        mCollecting = new Collecting();
        mVision = new Vision(Configuration.s_Current.getLimelight("limelight"), hardwareMap, "vision", telemetry);
        mVision.initialize();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        Pose2d updatedPose = null;
        mDrive = new MecanumDrive(hardwareMap, beginPose);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        double x_shooting_pos = 74;
        double y_shooting_pos = 0;
        boolean changePOS = true ;

        waitForStart();


            Vision.Pattern pattern = mVision.readPattern();
            telemetry.addLine("Pattern is " + pattern.text());
            if (mVision.readPattern() == Vision.Pattern.GPP){
                Actions.runBlocking(
                        mDrive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(30,32), Math.PI/2)
                                .build());
            }
            if (mVision.readPattern() == Vision.Pattern.PGP){
                Actions.runBlocking(
                mDrive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(55,32), Math.PI/2)
                        .build());
            }
            if (mVision.readPattern() == Vision.Pattern.PPG){
                Actions.runBlocking(
                        mDrive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(70,32), Math.PI/2)
                                .build());
            }
            mCollecting.intake();


            Actions.runBlocking(
                    mDrive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(x_shooting_pos ,y_shooting_pos), Math.PI/4)
                            .build());


            Pose3D output = mVision.getPosition();

            if (output != null) {
                telemetry.addLine("april tag pos not null");
                double x_from_april_tag = (-output.getPosition().x)*39.37;
                double y_from_april_tag = (-output.getPosition().y)*39.37;
                double heading_from_april_tag = -output.getOrientation().getYaw() + 180;
                while (heading_from_april_tag < -180) {
                    heading_from_april_tag += 360;
                }
                while (heading_from_april_tag > 180) {
                    heading_from_april_tag -= 360;
                }

               if (changePOS ){
                    updatedPose = new Pose2d(x_from_april_tag, y_from_april_tag, (heading_from_april_tag) * Math.PI / 180);
                    mDrive.changeLocalizer(updatedPose);
                    telemetry.addData("Pos changed to: ",updatedPose ) ;
                    changePOS=false;
                }


                telemetry.addData("X from april tag", x_from_april_tag);
                telemetry.addData("Y from april tag", y_from_april_tag);
                telemetry.addData("Heading from april tag", heading_from_april_tag);
                telemetry.update() ;

                Actions.runBlocking(
                        mDrive.actionBuilder(updatedPose)
                                .waitSeconds(10)
                                .build());
//                Actions.runBlocking(
//                        mDrive.actionBuilder(updatedPose)
//                                .splineTo(new Vector2d(24, 24), Math.PI / 2)
//                                .build());
//                telemetry.addData("New Pose: ", mDrive.updatePoseEstimate());
//                telemetry.update() ;
//                mCollecting.shooting();

//                if (mVision.readPattern() != Vision.Pattern.PPG) {
//                    Actions.runBlocking(
//                            mDrive.actionBuilder(beginPose)
//                                    .splineTo(new Vector2d(50, 12), Math.PI / 2)
//                                    .build());
//                } else {
//                    Actions.runBlocking(
//                            mDrive.actionBuilder(beginPose)
//                                    .splineTo(new Vector2d(50, -12), Math.PI / 2)
//                                    .build());
//                }
//                mCollecting.intake();

//                Actions.runBlocking(
//                        mDrive.actionBuilder(beginPose)
//                                .splineTo(new Vector2d(x_shooting_pos, y_shooting_pos), Math.PI / 4)
//                                .build());

                mCollecting.shooting();
            }

            mVision.close();


        }
    }


