package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@Autonomous(name = "AAA_Auto (Gripper)", group = "Autonomous")
public class jeff_auto extends LinearOpMode {

    final int SLIDE_GROUND = 0;
    final int SLIDE_CATCH = 500;
    final int SLIDE_HALF = 1350;
    final int SLIDE_HIGH = 2650;
    final double SLIDE_STALL_TIME = 2.0;

    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_DEPOSIT = 95 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BUCKET = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_PREPARE_TO_COLLECT = 180 * ARM_TICKS_PER_DEGREE; // parallel to the ground
    final double ARM_COLLECT = 190 * ARM_TICKS_PER_DEGREE;
    final double ARM_RAISE_ABOVE_GROUND = 195 * ARM_TICKS_PER_DEGREE;

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

    public class Slide {
        private DcMotorEx leftSlide;
        private DcMotorEx rightSlide;

        public Slide(HardwareMap hardwareMap) {
            leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftSlide.setPower(1.0);
            rightSlide.setPower(1.0);
            leftSlide.setTargetPosition(SLIDE_GROUND);
            rightSlide.setTargetPosition(SLIDE_GROUND);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public class SlidesUpHigh implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide < SLIDE_HIGH) {
                    leftSlide.setTargetPosition(SLIDE_HIGH);
                    rightSlide.setTargetPosition(SLIDE_HIGH);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesUpHigh() {
            return new SlidesUpHigh();
        }

        public class SlidesDownGround implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide > (SLIDE_GROUND + 100)) {
                    leftSlide.setTargetPosition(SLIDE_GROUND);
                    rightSlide.setTargetPosition(SLIDE_GROUND);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownGround() {
            return new SlidesDownGround();
        }

        public class SlidesDownHalf implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide > (SLIDE_HALF + 100)) {
                    leftSlide.setTargetPosition(SLIDE_HALF);
                    rightSlide.setTargetPosition(SLIDE_HALF);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownHalf() {
            return new SlidesDownHalf();
        }

        public class SlidesDownCatch implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setPower(1.0);
                    rightSlide.setPower(1.0);
                    initialized = true;
                }

                double posLeftSlide = rightSlide.getCurrentPosition();
                double posRightSlide = rightSlide.getCurrentPosition();
                packet.put("posLeftSlide", posLeftSlide);
                packet.put("posRightSlide", posRightSlide);
                if (posRightSlide > (SLIDE_CATCH)) {
                    leftSlide.setTargetPosition(SLIDE_CATCH);
                    rightSlide.setTargetPosition(SLIDE_CATCH);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action SlidesDownCatch() {
            return new SlidesDownCatch();
        }
    }

    public class Arm {
        private DcMotorEx armMotor;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "arm");
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) armMotor).setVelocity(2100);
        }

        public class ArmCollect implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_COLLECT - 5) {  // 5 is the buffer to avoid delay
                    armMotor.setTargetPosition((int) ARM_COLLECT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmCollect() {
            return new ArmCollect();
        }

        public class ArmDeposit implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos > ARM_DEPOSIT) {
                    armMotor.setTargetPosition((int) ARM_DEPOSIT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmDeposit() {
            return new ArmDeposit();
        }

        public class ArmClearBucket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_CLEAR_BUCKET) {
                    armMotor.setTargetPosition((int) ARM_CLEAR_BUCKET);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmClearBucket() {
            return new ArmClearBucket();
        }

        public class ArmCollapsedIntoRobot implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos > ARM_COLLAPSED_INTO_ROBOT) {
                    armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmCollapsedIntoRobot() {
            return new ArmCollapsedIntoRobot();
        }

        public class ArmPrepareToCollect implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos < ARM_PREPARE_TO_COLLECT) {
                    armMotor.setTargetPosition((int) ARM_PREPARE_TO_COLLECT);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmPrepareToCollect() {
            return new ArmPrepareToCollect();
        }

        public class ArmRaiseAboveGround implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(1.0);
                    initialized = true;
                }

                double pos = armMotor.getCurrentPosition();
                packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
                if (pos > ARM_RAISE_ABOVE_GROUND + 2) { // 2 is the buffer to avoid delay
                    armMotor.setTargetPosition((int) ARM_RAISE_ABOVE_GROUND);
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action ArmRaiseAboveGround() {
            return new ArmRaiseAboveGround();
        }

    }

    public class Bucket {
        private Servo bucket;

        public Bucket(HardwareMap hardwareMap) {
            bucket = hardwareMap.get(Servo.class, "bucket");
        }

        public class BucketDump implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(BUCKET_DUMP);
                packet.put("BucketPos", bucket.getPosition());
                return false;
            }
        }

        public Action BucketDump() {
            return new BucketDump();
        }

        public class BucketCatch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                bucket.setPosition(BUCKET_CATCH);
                packet.put("BucketPos", bucket.getPosition());
                return false;
            }
        }

        public Action BucketCatch() {
            return new BucketCatch();
        }
    }

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
            return new ElbowDeposit();
        }

        public class ElbowCollect implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setPosition(ELBOW_COLLECT);
                return false;
            }
        }

        public Action ElbowCollect() {
            return new ElbowCollect();
        }
    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }

        public class GripperIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_IN);
                return false;
            }
        }

        public Action GripperIn() {
            return new GripperIn();
        }

        public class GripperGrabOutwards implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_GRABBING_OUTWARDS);
                return false;
            }
        }

        public Action GripperGrabOutwards() {
            return new GripperGrabOutwards();
        }

        public class GripperGrabInwards implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_GRABBING_INWARDS);
                return false;
            }
        }

        public Action GripperGrabInwards() {
            return new GripperGrabInwards();
        }

        public class GripperOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(GRIPPER_OUT);
                return false;
            }
        }

        public Action GripperOut() {
            return new GripperOut();
        }
    }

    public class Flag {
        private Servo flag;

        public Flag(HardwareMap hardwareMap) {
            flag = hardwareMap.get(Servo.class, "flag");
        }

        public class FlagDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flag.setPosition(FLAG_DOWN);
                return false;
            }
        }

        public Action FlagDown() {
            return new FlagDown();
        }

        public class FlagScore implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                flag.setPosition(FLAG_SCORE);
                return false;
            }
        }

        public Action FlagScore() {
            return new FlagScore();
        }
    }

    public class Headlight {
        private Servo headlight;

        public Headlight(HardwareMap hardwareMap) {
            headlight = hardwareMap.get(Servo.class, "headlight");
        }

        public class headlight_Off implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                headlight.setPosition(HEADLIGHT_OFF);
                return false;
            }
        }

        public Action headlight_Off() {
            return new headlight_Off();
        }

        public class headlight_On implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                headlight.setPosition(HEADLIGHT_ON);
                return false;
            }
        }

        public Action headlight_On() {
            return new headlight_On();
        }
    }

    public class IndicatorLight {
        private Servo indicatorlight;

        public IndicatorLight(HardwareMap hardwareMap) {
            indicatorlight = hardwareMap.get(Servo.class, "indicator_light");
        }

        public class TurnIndicatorLight_Off implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                indicatorlight.setPosition(INDICATOR_LIGHT_OFF);
                return false;
            }
        }

        public Action TurnIndicatorLight_Off() {
            return new TurnIndicatorLight_Off();
        }

        public class TurnIndicatorLight_Green implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                indicatorlight.setPosition(INDICATOR_LIGHT_GREEN);
                return false;
            }
        }

        public Action TurnIndicatorLight_Green() {
            return new TurnIndicatorLight_Green();
        }
    }

    public class LimeLightVision {
        private Limelight3A limelight3A;

        public LimeLightVision(HardwareMap hardwareMap) {
            limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
            limelight3A.start();
        }

        public void ObtainCrosshair() {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX);

            LLStatus status = limelight3A.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult limelight_result = limelight3A.getLatestResult();

            if (limelight_result != null) {
                // Access general information
                Pose3D botpose = limelight_result.getBotpose();
                double captureLatency = limelight_result.getCaptureLatency();
                double targetingLatency = limelight_result.getTargetingLatency();
                double parseLatency = limelight_result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(limelight_result.getPythonOutput()));

                if (limelight_result.isValid()) {
                    telemetry.addData("tx", limelight_result.getTx());
                    telemetry.addData("txnc", limelight_result.getTxNC());
                    telemetry.addData("ty", limelight_result.getTy());
                    telemetry.addData("tync", limelight_result.getTyNC());
                    telemetry.addData("Botpose", botpose.toString());

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = limelight_result.getColorResults();
                    LLResultTypes.ColorResult cr = colorResults.get(0);

                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());

//                    crosshair_x = 8 * Math.tan(cr.getTargetXDegrees());
                    crosshair_x = cr.getTargetXDegrees() * ANGLE_TO_DISTANCE_FACTOR;
                    crosshair_y = cr.getTargetYDegrees() * ANGLE_TO_DISTANCE_FACTOR;
                    crosshair_angle = 0;

                    telemetry.addData("Crosshair", "X: %.2f, Y: %.2f", crosshair_x, crosshair_y);
                }
            } else {
                telemetry.addData("Limelight", "No data available");            }

            telemetry.update();
            limelight3A.stop();
        }
   }

    public class DriveBase {
        public DriveBase(HardwareMap hardwareMap) {
        }

        public class AlignToTarget_X implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pose2d initialPose = new Pose2d(0, 0, 0);
                MecanumDrive bot = new MecanumDrive(hardwareMap, initialPose);
                LimeLightVision limelight = new LimeLightVision(hardwareMap);
                double distance_to_target_x;
                double distance_to_target_y;

                limelight.ObtainCrosshair();
                distance_to_target_x = 0;
                distance_to_target_y = -crosshair_x;

                TrajectoryActionBuilder trajDriveToTarget = bot.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(distance_to_target_x, distance_to_target_y));
                Action actDriveToTarget = trajDriveToTarget.build();

                Actions.runBlocking(
                        new SequentialAction(
                                actDriveToTarget));
                return false;
            }
        }

        public Action AlignToTarget_X() {
            return new AlignToTarget_X();}

        public class MoveBackToToInitialPose_X implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Pose2d initialPose = new Pose2d(0, 0, 0);
                MecanumDrive bot = new MecanumDrive(hardwareMap, initialPose);

                double distance_to_target_x;
                double distance_to_target_y;

                distance_to_target_x = 0;
                distance_to_target_y = crosshair_x;

                TrajectoryActionBuilder trajDriveToTarget = bot.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(distance_to_target_x, distance_to_target_y));
                Action actDriveToTarget = trajDriveToTarget.build();

                Actions.runBlocking(
                        new SequentialAction(
                                actDriveToTarget));
                return false;
            }
        }

        public Action MoveBackToToInitialPose_X() {
            return new MoveBackToToInitialPose_X();
        }
    }

    public class FTCTelemetry {
        double auto_starttime;
        double total_duration;

        public FTCTelemetry(HardwareMap hardwareMap) {
        }

        public class ResetTimer implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                auto_starttime = System.currentTimeMillis();
                return false;
            }
        }

        public Action ResetTimer() {
            return new ResetTimer();
        }

        public class UpdateTotalDuration implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                total_duration = (System.currentTimeMillis() - auto_starttime) / 1000;
                packet.put("Total    Duration", total_duration);
                return false;
            }
        }

        public Action UpdateTotalDuration() {
            return new UpdateTotalDuration();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Slide slide = new Slide(hardwareMap);
        Bucket bucket = new Bucket(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);
        Flag flag = new Flag(hardwareMap);
        Headlight headlight = new Headlight(hardwareMap);
        IndicatorLight indicatorlight = new IndicatorLight(hardwareMap);
        FTCTelemetry ftctelemetry = new FTCTelemetry(hardwareMap);
        DriveBase drivebase = new DriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToHighBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-44, -60));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition1 = trajDriveToHighBasket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51, -46), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket2 = trajDriveToCollectSamplePosition1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -47), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition2 = trajDriveToHighBasket2.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-61, -46), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket3 = trajDriveToCollectSamplePosition2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -47), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition3 = trajDriveToHighBasket3.endTrajectory().fresh()
                .turnTo(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(-61, -45), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket4 = trajDriveToCollectSamplePosition3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-56, -47), Math.toRadians(45));

        TrajectoryActionBuilder trajDriveToPark = trajDriveToHighBasket4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(0)), 0)
                .lineToXConstantHeading(-31);

        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }

        Action actDriveToHighBasket = trajDriveToHighBasket.build();
        Action actDriveToCollectSamplePosition1 = trajDriveToCollectSamplePosition1.build();
        Action actDriveToHighBasket2 = trajDriveToHighBasket2.build();
        Action actDriveToCollectSamplePosition2 = trajDriveToCollectSamplePosition2.build();
        Action actDriveToHighBasket3 = trajDriveToHighBasket3.build();
        Action actDriveToCollectSamplePosition3 = trajDriveToCollectSamplePosition3.build();
        Action actDriveToHighBasket4 = trajDriveToHighBasket4.build();
        Action actDriveToPark = trajDriveToPark.build();

        waitForStart();

        this.resetRuntime();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
// This is for Limelight testing and determining the conversion factor.
//                        slide.SlidesUpHigh(),
//                        arm.ArmPrepareToCollect(),
//                        gripper.GripperOut(),
//                        drivebase.AlignToTarget_X()
//                        gripper.GripperGrabInwards(),
//                        drivebase.MoveBackToToInitialPose_X()

                        new ParallelAction(
                                ftctelemetry.ResetTimer(),
                                headlight.headlight_Off(),
                                indicatorlight.TurnIndicatorLight_Off()
                        ),

                        // Score preloaded sample to high basket
                        new ParallelAction(
                                bucket.BucketCatch(),
                                slide.SlidesUpHigh(),
                                actDriveToHighBasket,
                                elbow.ElbowCollect()
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 1st sample from mat
                        new ParallelAction(
                                arm.ArmPrepareToCollect(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                headlight.headlight_On(),
                                slide.SlidesDownCatch(),
                                actDriveToCollectSamplePosition1,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollect(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        arm.ArmRaiseAboveGround(),
                        drivebase.MoveBackToToInitialPose_X(),
                        new ParallelAction(
                                actDriveToHighBasket2,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                elbow.ElbowDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        gripper.GripperOut(),
                                        arm.ArmClearBucket(),
                                        slide.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 2nd sample from mat
                        new ParallelAction(
                                arm.ArmPrepareToCollect(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                headlight.headlight_On(),
                                slide.SlidesDownCatch(),
                                actDriveToCollectSamplePosition2,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollect(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        arm.ArmRaiseAboveGround(),
                        drivebase.MoveBackToToInitialPose_X(),
                        new ParallelAction(
                                actDriveToHighBasket3,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                elbow.ElbowDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        gripper.GripperOut(),
                                        arm.ArmClearBucket(),
                                        slide.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 3rd sample from mat
                        new ParallelAction(
                                arm.ArmPrepareToCollect(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                headlight.headlight_On(),
                                slide.SlidesDownCatch(),
                                actDriveToCollectSamplePosition3,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollect(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        arm.ArmRaiseAboveGround(),
                        drivebase.MoveBackToToInitialPose_X(),
                        new ParallelAction(
                                actDriveToHighBasket4,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                elbow.ElbowDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        gripper.GripperOut(),
                                        arm.ArmClearBucket(),
                                        slide.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        new ParallelAction(
                            bucket.BucketCatch(),
                            arm.ArmCollapsedIntoRobot(),
                            slide.SlidesDownGround(),
                            actDriveToPark,
                            flag.FlagScore()
                        ),
                        ftctelemetry.UpdateTotalDuration()
                )
        );
    }
}