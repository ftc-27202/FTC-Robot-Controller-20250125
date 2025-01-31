/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "01_TeleOp (Red Alliance)", group = "Robot")
//@Disabled

public class jeff_TeleOp_red_alliance extends LinearOpMode {
    final String allianceColor = "RED";  // Valid Values: RED or BLUE
    //    final int AllianceColor = "BLUE";  // Valid Values: RED or BLUE

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Wrist wrist = new Bot_Wrist(hardwareMap);
        Bot_Gripper gripper = new Bot_Gripper(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        Bot_Drivebase drivebase = new Bot_Drivebase(hardwareMap, allianceColor);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //if left_trigger: speed = 0.6; else speed = 1.0
            double speed = gamepad1.right_trigger > 0 ? 0.6 : 1.0;
            double turn_speed = gamepad1.right_trigger > 0 ? 0.2 : 1.0;
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x * turn_speed;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speed);
            rightFrontDrive.setPower(rightFrontPower * speed);
            leftBackDrive.setPower(leftBackPower * speed);
            rightBackDrive.setPower(rightBackPower * speed);


            if (gamepad1.left_trigger > 0) {
                // prepare to collect (either sample of specimen), from starting position
                runningActions.add(new ParallelAction(
                            flag.FlagDown(),
                            headlight.headlight_Off(),
                            indicatorlight.TurnIndicatorLight_Off(),
                            bucket.BucketDump(),
                            gripper.GripperOut(),
                            new SequentialAction(
                                new ParallelAction(
                                    slides.SlidesClearArm(),
                                    new SequentialAction(
                                    wrist.WristCollect(),
                                    new SleepAction(1.0))),
                                arm.ArmPrepareToCollect())));
            } else if (gamepad1.x) {
                // prepare to collect (either sample of specimen)
                runningActions.add(new ParallelAction(
                        flag.FlagDown(),
                        bucket.BucketOff(),
                        gripper.GripperOut(),
                        wrist.WristCollect(),
                        slides.SlidesDownGround(),
                        arm.ArmPrepareToCollect()));
            } else if (gamepad1.a) {
                // collect sample option 1: gripper out to in
                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                gripper.GripperOut(),
                                new SequentialAction(
                                        drivebase.AlignToNeutralSample_X(),
                                        wrist.WristCollect(),
                                        arm.ArmCollectSample(),
                                        gripper.GripperGrabInwards(),
                                        new SleepAction(0.2),
                                        arm.ArmCollected())
                        )
                ));
            } else if (gamepad1.b) {
                // collect specimen
                runningActions.add(new SequentialAction(
                        // use limelight???
                        new ParallelAction(
                                gripper.GripperOut(),
                                wrist.WristCollect(),
                                slides.SlidesDownGround(),
                                new SequentialAction(
                                        arm.ArmCollectSpecimen(),
                                        gripper.GripperGrabInwards(),
                                        new SleepAction(0.2),
                                        arm.ArmCollected())
                        )
                ));
            } else if (gamepad1.y) {
                // collect sample option 2: gripper halfway open
                runningActions.add(new SequentialAction(
                        // use limelight???
                        new ParallelAction(
                                gripper.GripperOpenHalfway(),
                                new SequentialAction(
                                        wrist.WristCollect(),
                                        arm.ArmCollectSample(),
                                        gripper.GripperGrabInwards(),
                                        new SleepAction(0.2),
                                        arm.ArmCollected())
                        )
                ));
            } else if (gamepad1.dpad_down) {
                // deposit sample to bucket
                runningActions.add(new ParallelAction(
                        bucket.BucketCatch(),
                        wrist.WristDeposit(),
                        slides.SlidesUpCatch(),
                        new SequentialAction(
                            arm.ArmDeposit(),
                            gripper.GripperOut()
                        )
                ));
            } else if (gamepad1.dpad_up) {
                // Prepare Sample to Score in High Basket
                runningActions.add(new SequentialAction(
                        arm.ArmClearBucket(),
                        slides.SlidesUpHigh()
                ));
            } else if (gamepad1.dpad_left) {
                // Score Specimen
                runningActions.add(new ParallelAction(
                        slides.SlidesDownGround(),
                        arm.ArmUpSpecimenBeforeScore()
                ));
            } else if (gamepad1.right_bumper) {
                // Dump Bucket
                runningActions.add(new SequentialAction(
                        bucket.BucketDump()
                ));
            }

            //        // Changed to wait for both triggers
            //        if (gamepad2.a && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            //            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //        }
            //
            //        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            //            telemetry.addData("Target Position", leftSlide.getTargetPosition());
            //            telemetry.addData("Actual Position", leftSlide.getCurrentPosition());
            //            slideTargetPosition -= (int) (gamepad2.right_stick_y * 30.0);
            //        }
            //
            //        // Added arm reset for league qualifier
            //        if (gamepad2.b && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            //            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            //        }
            //
            //        // Added arm reset for league qualifier
            //        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            //            telemetry.addData("Arm Target Position", armMotor.getTargetPosition());
            //            telemetry.addData("Arm Current Position", armMotor.getCurrentPosition());
            //            armPosition -= gamepad2.left_stick_y * ARM_TICKS_PER_DEGREE * 5.0;
            //        }
            //        if (gamepad2.left_bumper){
            //            flag.setPosition(FLAG_DOWN);
            //        }else if (gamepad2.right_bumper){
            //            flag.setPosition(FLAG_SCORE);
            //        }

            //        //slides not in position
            //        if (getRuntime() >= lastSlideActionTime + SLIDE_STALL_TIME) {
            //            final double leftSlideRemaining = Math.abs(leftSlide.getTargetPosition() - leftSlide.getCurrentPosition());
            //            final double rightSlideRemaining = Math.abs(rightSlide.getTargetPosition() - rightSlide.getCurrentPosition());
            //
            //            if (leftSlideRemaining > 200 || rightSlideRemaining > 200) {
            //                leftSlide.setPower(0.0);
            //                rightSlide.setPower(0.0);
            //                telemetry.addLine("SLIDE(S) STUCK!");
            //                return;
            //            }
            //        }
            //

            //        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.left_trigger);
            //        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            //        ((DcMotorEx) armMotor).setVelocity(2100);
            //        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            //prevents extensions being 42 inches or more
//            if (armMotor.getTargetPosition() > ARM_SCORE_SPECIMEN && (leftSlide.getTargetPosition() > SLIDE_HALF || rightSlide.getTargetPosition() > SLIDE_HALF)) {
//                slideTargetPosition = SLIDE_HALF;
//            }

//        Add code to prevent the bot from tipping over.

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            // Show the wheel power.
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}