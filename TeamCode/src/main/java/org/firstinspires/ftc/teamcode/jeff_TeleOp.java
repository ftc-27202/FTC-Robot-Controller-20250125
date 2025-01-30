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
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AA_TeleOp (Red Alliance)", group = "Robot")
//@Disabled

public class jeff_TeleOp extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

//    final int SLIDE_GROUND = 0;
//    final int SLIDE_CATCH = 500;
//    final int SLIDE_CLEAR_ARM = 1350;
//    final int SLIDE_HIGH = 2650;
//    final double SLIDE_STALL_TIME = 2.0;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_DEPOSIT = 93 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BUCKET = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_PREPARE_TO_COLLECT = 180 * ARM_TICKS_PER_DEGREE; // parallel to the ground
    final double ARM_COLLECT = 188 * ARM_TICKS_PER_DEGREE;

    final double BUCKET_CATCH = 0.5;
    final double BUCKET_DUMP = 0.1;

    // For physical install, 0.0 = is facing upwards (viewpoint when collecting), slightly off due to BWT link servo block hole placement issue
    final double ELBOW_DEPOSIT = 0.32;
    final double ELBOW_COLLECT = 0.99;

    // For physical install, 0.5 = Gripper middle position
    final double GRIPPER_IN = 0;
    final double GRIPPER_GRABBING_INWARDS = 0.25;
    final double GRIPPER_GRABBING_OUTWARDS = 1.0;
    final double GRIPPER_OUT = 1.0;

    // For physical install, 1.0 = Flag is facing all the way down
    final double FLAG_DOWN = 1.0;
    final double FLAG_SCORE = 0.30;

    final double HEADLIGHT_OFF = 0;
    final double HEADLIGHT_ON = 0.5;

    final double INDICATOR_LIGHT_OFF = 0;
    final double INDICATOR_LIGHT_GREEN = 0.5;

    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX = 7;
    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_RED_INDEX = 8;
    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_BLUE_INDEX = 9;

    final double ANGLE_TO_DISTANCE_FACTOR = 0.17;  // conversion for Limelight degrees to inches (very crude)

    double crosshair_x;
    double crosshair_y;
    double crosshair_angle;

//    public class Slide {
//        private DcMotorEx leftSlide;
//        private DcMotorEx rightSlide;
//
//        public Slide(HardwareMap hardwareMap) {
//            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
//            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
//            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
//            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////            leftSlide.setPower(1.0);
////            rightSlide.setPower(1.0);
//            leftSlide.setTargetPosition(SLIDE_GROUND);
//            rightSlide.setTargetPosition(SLIDE_GROUND);
//            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//        public class SlidesUpHigh implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    leftSlide.setPower(1.0);
//                    rightSlide.setPower(1.0);
//                    initialized = true;
//                }
//
//                double posLeftSlide = rightSlide.getCurrentPosition();
//                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
//                if (posRightSlide < SLIDE_HIGH) {
//                    leftSlide.setTargetPosition(SLIDE_HIGH);
//                    rightSlide.setTargetPosition(SLIDE_HIGH);
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action SlidesUpHigh() {
//            return new Slide.SlidesUpHigh();
//        }
//
//        public class SlidesDownGround implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    leftSlide.setPower(1.0);
//                    rightSlide.setPower(1.0);
//                    initialized = true;
//                }
//
//                double posLeftSlide = rightSlide.getCurrentPosition();
//                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
//                if (posRightSlide > (SLIDE_GROUND + 100)) {
//                    leftSlide.setTargetPosition(SLIDE_GROUND);
//                    rightSlide.setTargetPosition(SLIDE_GROUND);
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action SlidesDownGround() {
//            return new SlidesDownGround();
//        }
//
//        public class SlidesClearArm implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    leftSlide.setPower(1.0);
//                    rightSlide.setPower(1.0);
//                    initialized = true;
//                }
//
//                double posLeftSlide = rightSlide.getCurrentPosition();
//                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
//                if (posRightSlide < (SLIDE_CLEAR_ARM - 100)) {
//                    leftSlide.setTargetPosition(SLIDE_CLEAR_ARM);
//                    rightSlide.setTargetPosition(SLIDE_CLEAR_ARM);
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action SlidesClearArm() {
//            return new SlidesClearArm();
//        }
//
//        public class SlidesDownCatch implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    leftSlide.setPower(1.0);
//                    rightSlide.setPower(1.0);
//                    initialized = true;
//                }
//
//                double posLeftSlide = rightSlide.getCurrentPosition();
//                double posRightSlide = rightSlide.getCurrentPosition();
//                packet.put("posLeftSlide", posLeftSlide);
//                packet.put("posRightSlide", posRightSlide);
//                if (posRightSlide > (SLIDE_CATCH)) {
//                    leftSlide.setTargetPosition(SLIDE_CATCH);
//                    rightSlide.setTargetPosition(SLIDE_CATCH);
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action SlidesDownCatch() {
//            return new SlidesDownCatch();
//        }
//
//    }
//
    public class Elbow {
        private Servo elbow;

        public Elbow(HardwareMap hardwareMap) {
            elbow = hardwareMap.get(Servo.class, "elbow");
        }

        public class ElbowDeposit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(ELBOW_DEPOSIT);
                return false;
            }
        }

        public Action ElbowDeposit() {
            return new Elbow.ElbowDeposit();
        }

        public class ElbowCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(ELBOW_COLLECT);
                return false;
            }
        }

        public Action ElbowCollect() {
            return new Elbow.ElbowCollect();
        }
    }

    @Override
    public void runOpMode() {
        TelemetryPacket packet = new TelemetryPacket();
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Elbow elbow = new Bot_Elbow(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightRear");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        DcMotorEx leftSlide;
//        DcMotorEx rightSlide;
//
//        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
//        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
//        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftSlide.setTargetPosition(SLIDE_GROUND);
//        rightSlide.setTargetPosition(SLIDE_GROUND);
//        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start(); // This tells Limelight to start looking!
//        limelight.pipelineSwitch(0); // Switch to pipeline number 0
//        headlight = hardwareMap.get(Servo.class, "headlight");
//        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
//        leftSlide.setDirection(DcMotor.Direction.FORWARD);
//        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
//        rightSlide.setDirection(DcMotor.Direction.REVERSE);
//        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//    jeff_auto.Slide slide = new jeff_auto.Slide(hardwareMap);
//    jeff_auto.Bucket bucket = new jeff_auto.Bucket(hardwareMap);
//    jeff_auto.Arm arm = new jeff_auto.Arm(hardwareMap);
//    jeff_auto.Gripper gripper = new jeff_auto.Gripper(hardwareMap);
//    jeff_auto.Flag flag = new jeff_auto.Flag(hardwareMap);
//    jeff_auto.Headlight headlight = new jeff_auto.Headlight(hardwareMap);
//    jeff_auto.IndicatorLight indicatorlight = new jeff_auto.IndicatorLight(hardwareMap);
//    jeff_auto.FTCTelemetry ftctelemetry = new jeff_auto.FTCTelemetry(hardwareMap);
//    jeff_auto.DriveBase drivebase = new jeff_auto.DriveBase(hardwareMap);


//    public DcMotor leftSlide;
//    public DcMotor rightSlide;
//    public Limelight3A limelight;
//    public DcMotor armMotor;
//    public CRServo intake;
//    public Servo wrist;
//    public Servo bucket;
//    public Servo elbow;
//    public Servo gripper;
//    public Servo flag;
//    public Servo headlight;

//    public void startStrafing(int power) {
//        leftFrontDrive.setPower(-power);
//        rightFrontDrive.setPower(power);
//        leftBackDrive.setPower(power);
//        rightBackDrive.setPower(-power);
//    }


//        armMotor = hardwareMap.get(DcMotor.class, "arm");
//        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////        intake = hardwareMap.get(CRServo.class, "intake");
////        intake.setPower(INTAKE_OFF);
////
////        wrist = hardwareMap.get(Servo.class, "wrist");
////        wrist.setPosition(WRIST_FOLDED_IN);
//
//        elbow = hardwareMap.get(Servo.class, "elbow");
//        flag = hardwareMap.get(Servo.class, "flag");
//        gripper = hardwareMap.get(Servo.class, "gripper");
//        bucket = hardwareMap.get(Servo.class, "bucket");

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

//                        slides.SlidesUpHigh())
            if (gamepad1.x) {
                runningActions.add(new SequentialAction(
                            elbow.ElbowCollect()
                ));
            } else if (gamepad1.y) {
                runningActions.add(new SequentialAction(
                        elbow.ElbowDeposit()
                ));
            }

//                leftSlide.setTargetPosition(SLIDE_CLEAR_ARM);
//                leftSlide.setPower(1.0);
//                leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                rightSlide.setTargetPosition(SLIDE_CLEAR_ARM);
//                rightSlide.setPower(1.0);
//                rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            ////            runningActions.add(new SequentialAction(
            ////                    slide.SlidesUpHigh()
            ////                    ,
            ////                    elbow.ElbowCollect()
            //            ));
            //        if (gamepad1.a) {
            //            intake.setPower(INTAKE_COLLECT);
            //            gripper.setPosition(GRIPPER_GRABBING_OUTWARDS);
            //        } else if (gamepad1.x) {
            //            intake.setPower(INTAKE_OFF);
            //        } else if (gamepad1.b) {
            //            intake.setPower(INTAKE_DEPOSIT);
            //            gripper.setPosition(GRIPPER_IN);
            //        }
            //
            //        if (gamepad1.right_bumper) {
            //            armPosition = ARM_COLLECT;
            //            wrist.setPosition(WRIST_FOLDED_OUT);
            //            intake.setPower(INTAKE_COLLECT);
            //            elbow.setPosition(ELBOW_COLLECT);
            //
            //        } else if (gamepad1.left_bumper) {
            //            wrist.setPosition(WRIST_FOLDED_OUT);
            //            armPosition = ARM_CLEAR_BARRIER;
            //        } else if (gamepad1.y) {
            //            armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            //        } else if (gamepad1.dpad_left) {
            //            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            //            intake.setPower(INTAKE_OFF);
            //            wrist.setPosition(WRIST_FOLDED_IN);
            //        } else if (gamepad1.dpad_right) {
            //            armPosition = ARM_SCORE_SPECIMEN;
            //            wrist.setPosition(WRIST_SPECIMEN);
            //        } else if (gamepad1.dpad_up) {
            //            armPosition = ARM_DEPOSIT;
            //            wrist.setPosition(WRIST_FOLDED_IN);
            //            elbow.setPosition(ELBOW_DEPOSIT);
            //        } else if (gamepad1.dpad_down) {
            //            armPosition = ARM_WINCH_ROBOT;
            //            intake.setPower(INTAKE_OFF);
            //            wrist.setPosition(WRIST_FOLDED_IN);
            //        }
            //
            //        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.left_trigger);
            //
            //        armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            //
            //        ((DcMotorEx) armMotor).setVelocity(2100);
            //        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            //
            //        if (gamepad2.dpad_up) {
            //            slideTargetPosition = SLIDE_HIGH;
            //            lastSlideActionTime = getRuntime();
            //        } else if (gamepad2.dpad_down) {
            //            slideTargetPosition = SLIDE_GROUND;
            //            lastSlideActionTime = getRuntime();
            //        } else if (gamepad2.dpad_left) {
            //            slideTargetPosition = SLIDE_HALF;
            //            lastSlideActionTime = getRuntime();
            //        }
            //
            //        if (gamepad2.left_trigger > 0) {
            //            bucket.setPosition(BUCKET_CATCH);
            //        } else if (gamepad2.right_trigger > 0) {
            //            bucket.setPosition(BUCKET_DUMP);
            //        }
            //
            //        if (gamepad2.y) {
            //            limelight.pipelineSwitch(0);
            //        } else if (gamepad2.b) {
            //            limelight.pipelineSwitch(1);
            //        } else if (gamepad2.x) {
            //            limelight.pipelineSwitch(2);
            //        }
            //
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
            //        //Limelight stuff starts here
            //        LLResult result = limelight.getLatestResult();
            //
            //        //gamepad2.dpad_right will target the robot to a seen sample (rn only yellow works)
            //        // if gamepad2.dpad_right is pressed and target is seen, identify direction and try to move towards the target until target is within tolerance
            //        if (gamepad2.dpad_right && (result != null && result.isValid()) ) {
            //            //gets results from LL
            //            double tx = result.getTx(); // How far left or right the target is using robot coords (degrees)
            //            double ty = result.getTy(); // How far up or down the target is (degrees)
            //            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            //
            //            telemetry.addData("Target X", tx);
            //            telemetry.addData("Target Y", ty);
            //            telemetry.addData("Target Area", ta);
            //            //move left or right based on data
            //            //Target tolerance is how many units the centre of the target has to be offset for the robot to decide to move
            //            // Right now it is arbitrarily set at 5, but can be changed, IDK what it should actually be
            //            //Set as constant in jeff base teleop
            //            //right is positive I think?
            //            if (tx >= LLTargetTolerance ){
            //                telemetry.addData("Move Right", tx);
            //                leftFrontDrive.setPower(LLSPEED*1.5);
            //                rightFrontDrive.setPower(-LLSPEED*1.5);
            //                leftBackDrive.setPower(-LLSPEED*1.5);
            //                rightBackDrive.setPower(LLSPEED*1.5);
            //            }else if((-1*LLTargetTolerance) >= tx){
            //                telemetry.addData("Move Left", tx);
            //                // Fill in with code to actually strafe the robot slowly to the left
            //                leftFrontDrive.setPower(-LLSPEED*1.5);
            //                rightFrontDrive.setPower(LLSPEED*1.5);
            //                leftBackDrive.setPower(LLSPEED*1.5);
            //                rightBackDrive.setPower(-LLSPEED*1.5);
            //            }else if(ty >= LLTargetTolerance){
            //                telemetry.addData("Move Forward", ty);
            //                leftFrontDrive.setPower(LLSPEED);
            //                rightFrontDrive.setPower(LLSPEED);
            //                leftBackDrive.setPower(LLSPEED);
            //                rightBackDrive.setPower(LLSPEED);
            //
            //
            //            }else if((-1*LLTargetTolerance) >= ty){
            //                telemetry.addData("Move Backward", ty);
            //                leftFrontDrive.setPower(-LLSPEED);
            //                rightFrontDrive.setPower(-LLSPEED);
            //                leftBackDrive.setPower(-LLSPEED);
            //                rightBackDrive.setPower(-LLSPEED);
            //
            //            }else if((Math.abs(tx)) < LLTargetTolerance && (Math.abs(ty)) < LLTargetTolerance){
            //                telemetry.addData("Target within tolerance, current offset:", tx);
            //                leftFrontDrive.setPower(0);
            //                rightFrontDrive.setPower(0);
            //                leftBackDrive.setPower(0);
            //                rightBackDrive.setPower(0);
            //            }
            //
            //        } else if (result != null && result.isValid()) {
            //            // if LL detects target but right dpad not pressed, just display target in telemetry
            //            double tx = result.getTy(); // How far left or right the target is (degrees)
            //            double ty = result.getTx(); // How far up or down the target is (degrees)
            //            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            //
            //            telemetry.addData("Target X", tx);
            //            telemetry.addData("Target Y", ty);
            //            telemetry.addData("Target Area", ta);
            //        }  else if (gamepad2.dpad_right) {
            //            telemetry.addLine("No Targets Found"); //If gamepad is pressed but no result, telemetry says no targets
            //        } else {
            //            telemetry.addData("Limelight", "No Targets or LL not working"); //Also no targets even if gamepad not pressed
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
            //        //prevents extensions being 42 inches or more
            //        if (armMotor.getTargetPosition() > ARM_SCORE_SPECIMEN && (leftSlide.getTargetPosition() > SLIDE_HALF || rightSlide.getTargetPosition() > SLIDE_HALF)) {
            //            slideTargetPosition = SLIDE_HALF;
            //        }
            //
            //        leftSlide.setTargetPosition(slideTargetPosition);
            //        leftSlide.setPower(1.0);
            //        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            //        rightSlide.setTargetPosition(slideTargetPosition);
            //        rightSlide.setPower(1.0);
            //        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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