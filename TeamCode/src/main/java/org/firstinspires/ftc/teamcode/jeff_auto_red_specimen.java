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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
@Autonomous(name = "02_Auto (Red Specimen)", group = "Autonomous")
public class jeff_auto_red_specimen extends LinearOpMode {

    final int RED_ALLIANCE = 0;
    final int BLUE_ALLIANCE = 1;

    final int AllianceColor = RED_ALLIANCE;

    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX = 7;
    final int LIMELIGHT_PIPELINE_AUTO_RED_INDEX = 8;
    final int LIMELIGHT_PIPELINE_AUTO_BLUE_INDEX = 9;

    final double ANGLE_TO_DISTANCE_FACTOR = 0.17;  // conversion for Limelight degrees to inches (very crude)

    private double crosshair_x;
    private double crosshair_y;
    private double crosshair_angle;

    public class LimeLightVision {
        private Limelight3A limelight3A;

        public LimeLightVision(HardwareMap hardwareMap) {
            limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
            limelight3A.start();
        }

        public void ObtainCrosshair() {

            if (AllianceColor == RED_ALLIANCE) {
                limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_RED_INDEX);
            } else if (AllianceColor == BLUE_ALLIANCE){
                limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_BLUE_INDEX);}

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
        Pose2d initialPose = new Pose2d(6, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Elbow elbow = new Bot_Elbow(hardwareMap);
        Bot_Gripper gripper = new Bot_Gripper(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        FTCTelemetry ftctelemetry = new FTCTelemetry(hardwareMap);
        DriveBase drivebase = new DriveBase(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToSubmersible1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(-6, -34), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveBackToScoreSpecimen1 = trajDriveToSubmersible1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-6, -40), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToSample1 = trajDriveBackToScoreSpecimen1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(34, -46), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToSample2 = trajDriveToSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(44, -46), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToSample3 = trajDriveToSample2.endTrajectory().fresh()
                .turnTo(Math.toRadians(60))
                .splineToConstantHeading(new Vector2d(44, -44), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToDropSample3 = trajDriveToSample3.endTrajectory().fresh()
                .turnTo(Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToCollectSpecimen2 = trajDriveToDropSample3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(36, -50), Math.toRadians(0));

        TrajectoryActionBuilder trajDriveToSubmersible2 = trajDriveToCollectSpecimen2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-8, -34), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveBackToScoreSpecimen2 = trajDriveToSubmersible2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-8, -40), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToCollectSpecimen3 = trajDriveBackToScoreSpecimen2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(36, -50), Math.toRadians(0));

        TrajectoryActionBuilder trajDriveToSubmersible3 = trajDriveToCollectSpecimen3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-10, -34), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveBackToScoreSpecimen3 = trajDriveToSubmersible3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-10, -40), Math.toRadians(90));


        while (!isStopRequested() && !opModeIsActive()) {
//            telemetry.addData("x", drive.pose.position.x);
//            telemetry.addData("y", drive.pose.position.y);
//            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();
        }

        Action actDriveToSubmersible1 = trajDriveToSubmersible1.build();
        Action actDriveBackToScoreSpecimen1 = trajDriveBackToScoreSpecimen1.build();
        Action actDriveToSample1 = trajDriveToSample1.build();
        Action actDriveToSample2 = trajDriveToSample2.build();
        Action actDriveToSample3 = trajDriveToSample3.build();
        Action actDriveToDropSample3 = trajDriveToDropSample3.build();
        Action actDriveToCollectSpecimen2 = trajDriveToCollectSpecimen2.build();
        Action actDriveToSubmersible2 = trajDriveToSubmersible2.build();
        Action actDriveBackToScoreSpecimen2 = trajDriveBackToScoreSpecimen2.build();
        Action actDriveToCollectSpecimen3 = trajDriveToCollectSpecimen3.build();
        Action actDriveToSubmersible3 = trajDriveToSubmersible3.build();
        Action actDriveBackToScoreSpecimen3 = trajDriveBackToScoreSpecimen3.build();

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
                                indicatorlight.TurnIndicatorLight_Off(),
                                flag.FlagDown(),
                                elbow.ElbowCollect(),
                                bucket.BucketDump(),
                                gripper.GripperIn()
                        ),
                        new SleepAction(0.25),  // wait for the elbow to turn
                        new ParallelAction(
                                actDriveToSubmersible1,
                                new SequentialAction(
                                    slides.SlidesClearArm(),
                                    arm.ArmDownSpecimenBeforeScore())
                        ),
                        new ParallelAction(
                                actDriveBackToScoreSpecimen1,
                                arm.ArmSpecimenScore()
                        ),

                        // Drive to collect sample 1 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                arm.ArmPrepareToCollect(),
                                actDriveToSample1,
                                elbow.ElbowCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        arm.ArmDropSampleToZone(),
                        elbow.ElbowDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect sample 2 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                arm.ArmPrepareToCollect(),
                                actDriveToSample2,
                                elbow.ElbowCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        arm.ArmDropSampleToZone(),
                        elbow.ElbowDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect sample 3 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                arm.ArmPrepareToCollect(),
                                actDriveToSample3,
                                elbow.ElbowCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        actDriveToDropSample3,
                        arm.ArmDropSampleToZone(),
                        elbow.ElbowDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect specimen 2 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                actDriveToCollectSpecimen2,
                                arm.ArmPrepareToCollect(),
                                elbow.ElbowCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSpecimen(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        new ParallelAction(
                                actDriveToSubmersible2,
                                arm.ArmUpSpecimenBeforeScore()
                        ),
                        new ParallelAction(
                                actDriveBackToScoreSpecimen2,
                                arm.ArmSpecimenScore()
                        ),

                        // Drive to collect specimen 3 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                actDriveToCollectSpecimen3,
                                arm.ArmPrepareToCollect(),
                                elbow.ElbowCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSpecimen(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        new ParallelAction(
                                actDriveToSubmersible3,
                                arm.ArmUpSpecimenBeforeScore()
                        ),
                        new ParallelAction(
                                actDriveBackToScoreSpecimen3,
                                arm.ArmSpecimenScore()
                        ),

                        new SleepAction(5),  //temporary


//                        actDriveBackToScore1,
//                        gripper.GripperOut(),

//
//                        new ParallelAction(
//                                headlight.headlight_Off(),
//                                indicatorlight.TurnIndicatorLight_Off()
//                        ),
//
//                        // Score preloaded sample to high basket
//                        new ParallelAction(
//                                bucket.BucketCatch(),
//                                slide.SlidesUpHigh(),
//                                actDriveToHighBasket,
//                                elbow.ElbowCollect()
//                        ),
//                        bucket.BucketDump(),
//                        new SleepAction(0.4),
//
//                        // Drive to collect 1st sample from mat
//                        new ParallelAction(
//                                arm.ArmPrepareToCollect(),
//                                indicatorlight.TurnIndicatorLight_Off(),
//                                headlight.headlight_On(),
//                                slide.SlidesDownCatch(),
//                                actDriveToCollectSamplePosition1,
//                                elbow.ElbowCollect(),
//                                gripper.GripperOut(),
//                                bucket.BucketCatch()
//                        ),
//                        drivebase.AlignToTarget_X(),
//                        arm.ArmCollect(),
//                        gripper.GripperGrabInwards(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_X(),
//                        new ParallelAction(
//                                actDriveToHighBasket2,
//                                indicatorlight.TurnIndicatorLight_Green(),
//                                headlight.headlight_Off(),
//                                new SequentialAction(
//                                        new ParallelAction(
//                                                elbow.ElbowDeposit(),
//                                                arm.ArmDeposit()
//                                        ),
//                                        gripper.GripperOut(),
//                                        arm.ArmClearBucket(),
//                                        slide.SlidesUpHigh()
//                                )
//                        ),
//                        bucket.BucketDump(),
//                        new SleepAction(0.4),
//

                        ftctelemetry.UpdateTotalDuration()
                )
        );
    }
}