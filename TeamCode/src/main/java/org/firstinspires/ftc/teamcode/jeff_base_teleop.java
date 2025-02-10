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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.ArrayList;
import java.util.List;

public abstract class jeff_base_teleop extends LinearOpMode {
    private String allianceColor;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private Boolean ArmFudgeUpButtonPressed = false;
    private Boolean ArmFudgeDownButtonPressed = false;

    public void setAllianceColor(String inAllianceColor) {
        allianceColor = inAllianceColor;
    }
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

        double speed = 1.0;
        double turn_speed = 1.0;
        double max, axial, lateral, yaw;
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

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
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            axial = -gamepad1.left_stick_y + -(gamepad2.left_stick_y * 0.5);  // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x + (gamepad2.left_stick_x * 0.5);
            yaw = (gamepad1.right_stick_x + (gamepad2.right_stick_x * 0.5)) * turn_speed;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

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

            if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                // prepare to collect (either sample of specimen), from starting position
                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                flag.FlagDown(),
                                headlight.headlight_On(),
                                indicatorlight.TurnIndicatorLight_AllianceColor(allianceColor),
                                bucket.BucketDump(),
                                gripper.GripperOut()),
                        new SequentialAction(
                                new ParallelAction(
                                        slides.SlidesClearArm(),
                                        new SequentialAction(
                                                wrist.WristCollect(),
                                                new SleepAction(1)
                                        )
                                ),
                                arm.ArmPrepareToCollect()),
                        new SequentialAction(
                                bucket.BucketCatch(),
                                bucket.BucketOff()),
                        slides.SlidesDownGround())
                );
            } else if (gamepad1.dpad_down) {
                // deposit sample to bucket
                runningActions.add(new ParallelAction(
                        headlight.headlight_Off(),
                        bucket.BucketCatch(),
                        wrist.WristDeposit(),
                        slides.SlidesUpCatch(),
                        new SequentialAction(
                                arm.ArmDeposit(),
                                gripper.GripperOut(),
                                new SleepAction(0.2),
                                gripper.GripperIn()
                        )
                ));
            } else if (gamepad1.dpad_up) {
                // Prepare Sample to Score in High Basket
                speed = 0.40;
                turn_speed = 0.80;
                runningActions.add(new ParallelAction(
                        headlight.headlight_Off(),
                        new SequentialAction(
                                arm.ArmClearBucket(),
                                slides.SlidesUpHigh()
                        )));
            } else if (gamepad1.right_bumper && gamepad1.dpad_left) {
                // Prepare to Ascend
                runningActions.add(
                        new ParallelAction(
                                slides.SlidesUpAscend(),
                                wrist.WristDeposit(),
                                arm.ArmPrepareToAscend(),
                                bucket.BucketDump()));
            } else if (gamepad1.right_bumper && gamepad1.dpad_right) {
                // Ascend to Level 2
                runningActions.add(
                        new SequentialAction(
                                arm.ArmDownForAscending(),
                                new ParallelAction(
                                        slides.SlidesDownGround(),
                                        arm.ArmCollapsedIntoRobot()))
                );
            } else if (gamepad1.right_bumper) {
                // Dump Bucket
                runningActions.add(new SequentialAction(
                        bucket.BucketDump()
                ));
            } else if (gamepad1.left_bumper && gamepad1.a) {
                // Flag Down
                runningActions.add(new SequentialAction(
                        flag.FlagDown()
                ));
            } else if (gamepad1.a) {
                // Flag Raise / Score
                runningActions.add(new SequentialAction(
                        flag.FlagScore()
                ));
            } else if (gamepad1.left_bumper && gamepad1.b) {
                runningActions.add(new ParallelAction(
                        headlight.headlight_Off(),
                        indicatorlight.TurnIndicatorLight_Off()
                ));
            } else if (gamepad1.b) {
                runningActions.add(new ParallelAction(
                        headlight.headlight_On(),
                        indicatorlight.TurnIndicatorLight_AllianceColor(allianceColor)
                ));
            }
            ;

            // Gamepad 2 Controls
            if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.a) {
                // reset slides and arms
                runningActions.add(new ParallelAction(
                        slides.ResetSlides(),
                        arm.ArmReset(),
                        headlight.headlight_Off(),
                        new SleepAction(0.05),
                        headlight.headlight_On(),
                        new SleepAction(0.05),
                        headlight.headlight_Off()
                ));
            } else if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.y) {
                //arm fudge increase
                if (!ArmFudgeUpButtonPressed){
                runningActions.add(new SequentialAction(
                        arm.ArmFudgeUp(),
                        new SleepAction(0.05)
                ));}
                ArmFudgeUpButtonPressed = true;
            }else if (gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.x) {
               if (!ArmFudgeDownButtonPressed){
                   //arm fudge decrease
                runningActions.add(new SequentialAction(
                        arm.ArmFudgeDown(),
                        new SleepAction(0.05)
                ));}
                ArmFudgeDownButtonPressed = true;
            } else if (gamepad2.left_bumper && gamepad2.a) {
                // collect alliance sample: gripper out to in
                runningActions.add(new ParallelAction(
                        gripper.GripperOut(),
                        headlight.headlight_On(),
                        new SequentialAction(
                                drivebase.AlignToAllianceSample("VERTICAL"),
                                wrist.WristCollect(),
                                arm.ArmCollectSample(),
                                gripper.GripperIn(),
                                new SleepAction(0.2),
                                arm.ArmCollected())
                ));
            } else if (gamepad2.a) {
                // collect neutral sample: gripper out to in
                runningActions.add(new ParallelAction(
                        gripper.GripperOut(),
                        headlight.headlight_On(),
                        new SequentialAction(
                                drivebase.AlignToNeutralSample("VERTICAL"),
                                wrist.WristCollect(),
                                arm.ArmCollectSample(),
                                gripper.GripperIn(),
                                new SleepAction(0.2),
                                arm.ArmCollected())
                ));
            } else if (gamepad2.left_bumper && gamepad2.y) {
                // collect alliance sample: gripper in to out
                runningActions.add(new ParallelAction(
                        gripper.GripperIn(),
                        new SleepAction(0.2),
                        headlight.headlight_On(),
                        new SequentialAction(
                                drivebase.AlignToAllianceSample("HORIZONTAL"),
                                wrist.WristCollect(),
                                arm.ArmCollectSample(),
                                gripper.GripperOut(),
                                new SleepAction(0.2),
                                wrist.WristDiagonalDown(),
                                arm.ArmCollected())
                ));
            } else if (gamepad2.y) {
                // collect neutral sample: gripper in to out
                runningActions.add(new ParallelAction(
                        gripper.GripperIn(),
                        new SleepAction(0.2),
                        headlight.headlight_On(),
                        new SequentialAction(
                                drivebase.AlignToNeutralSample("HORIZONTAL"),
                                wrist.WristCollect(),
                                arm.ArmCollectSample(),
                                gripper.GripperOut(),
                                new SleepAction(0.2),
                                wrist.WristDiagonalDown(),
                                arm.ArmCollected())
                ));
            } else if (gamepad2.left_bumper) {
                // move slides, using the gamepad2.right_stick_y for desired adjustment
                runningActions.add(new SequentialAction(
                        slides.MoveSlides(-gamepad2.right_stick_y * 200)
                ));
            } else if (gamepad2.right_bumper) {
                // move arm, using the gamepad2.right_stick_y for desired adjustment
                runningActions.add(new SequentialAction(
                        arm.MoveArm(gamepad2.right_stick_y * 10)
                ));
            } else if (gamepad2.x) {
                // prepare to collect (either sample of specimen)
                speed = 1.0;
                turn_speed = 1.0;
                runningActions.add(new ParallelAction(
                        flag.FlagDown(),
                        headlight.headlight_On(),
                        new SequentialAction(
                                bucket.BucketCatch(),
                                bucket.BucketOff()),
                        gripper.GripperOut(),
                        slides.SlidesDownGround(),
                        new SequentialAction(
                                wrist.WristCollect(),
                                arm.ArmPrepareToCollect())));
            } else if (gamepad2.b) {
                // collect specimen
                runningActions.add(new ParallelAction(
                        gripper.GripperOut(),
                        headlight.headlight_On(),
                        new SequentialAction(
                                drivebase.AlignToSpecimen(),
                                wrist.WristCollect(),
                                arm.ArmCollectSpecimen(),
                                gripper.GripperIn(),
                                new SleepAction(0.2),
                                arm.ArmCollected())
                ));
            } else if (gamepad2.dpad_left) {
                // Prepare to Score Specimen
                runningActions.add(new ParallelAction(
                        slides.SlidesDownGround(),
                        arm.ArmUpSpecimenBeforeScore()
                ));
            } else if (gamepad2.dpad_right) {
                // Score Specimen
                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                arm.ArmSpecimenAfterScore(),
                                drivebase.MoveBackForSpecimen()),
                        new SleepAction(0.20),
                        gripper.GripperOut()
                ));
            } else if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                // return to from starting position
                runningActions.add(new SequentialAction(
                        new ParallelAction(
                                flag.FlagDown(),
                                headlight.headlight_Off(),
                                bucket.BucketDump(),
                                gripper.GripperIn()),
                        new SequentialAction(
                                new ParallelAction(
                                        slides.SlidesClearArm(),
                                        wrist.WristCollect()
                                ),
                                arm.ArmCollapsedIntoRobot()),
                        bucket.BucketCatch(),
                        bucket.BucketOff(),
                        slides.SlidesDownGround())
                );
            }
            ;
            if (!(gamepad2.left_bumper&& gamepad2.right_bumper && gamepad2.y)){
                ArmFudgeUpButtonPressed = false;
            }
            if (!(gamepad2.left_bumper&& gamepad2.right_bumper && gamepad2.x)){
                ArmFudgeDownButtonPressed = false;
            }

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
            //        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.left_trigger);
            //        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            //        ((DcMotorEx) armMotor).setVelocity(2100);
            //        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            //prevents extensions being 42 inches or more
//            if (armMotor.getTargetPosition() > ARM_SCORE_SPECIMEN && (leftSlide.getTargetPosition() > SLIDE_HALF || rightSlide.getTargetPosition() > SLIDE_HALF)) {
//                slideTargetPosition = SLIDE_HALF;
//            }

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            double ARM_FUDGE = arm.ARM_FUDGE;
            LLStatus LLStatusTelemetry = drivebase.getCameraStatus();
            LLResult LLResultTelemetry = drivebase.getCameraResult();
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);
            // Show the wheel power.
            if (LLStatusTelemetry!=null){
            telemetry.addData("Name", "%s",
                    LLStatusTelemetry.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    LLStatusTelemetry.getTemp(), LLStatusTelemetry.getCpu(), (int) LLStatusTelemetry.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    LLStatusTelemetry.getPipelineIndex(), LLStatusTelemetry.getPipelineType());
            } else{
                telemetry.addData("LimelightStatus ", "Null");
            }
            if (LLResultTelemetry!= null){
                if (LLResultTelemetry.isValid()) {
                    telemetry.addData("tx", LLResultTelemetry.getTx());
                    telemetry.addData("ty", LLResultTelemetry.getTy());
                }else{
                    telemetry.addData("Limelight", "No data available");
                }
            }else{
                telemetry.addData("LimelightResult", "Null");
            }
            telemetry.addData("packet", packet);
            telemetry.addData("Arm Fudged", ARM_FUDGE );
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}