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

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Config
@Autonomous(name = "01_Auto (Basket)", group = "Autonomous")
public class jeff_auto_basket extends LinearOpMode {

    //    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX = 7;
//    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_RED_INDEX = 8;
//    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_BLUE_INDEX = 9;
//
//    final double ANGLE_TO_DISTANCE_FACTOR = 0.17;  // conversion for Limelight degrees to inches (very crude)
//
//    double crosshair_x;
//    double crosshair_y;
//    double crosshair_angle;

//    public class LimeLightVision {
//        private Limelight3A limelight3A;
//
//        public LimeLightVision(HardwareMap hardwareMap) {
//            limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
//            limelight3A.start();
//        }
//
//        public void ObtainCrosshair() {
//            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX);
//
//            LLStatus status = limelight3A.getStatus();
//            telemetry.addData("Name", "%s",
//                    status.getName());
//            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                    status.getTemp(), status.getCpu(),(int)status.getFps());
//            telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                    status.getPipelineIndex(), status.getPipelineType());
//
//            LLResult limelight_result = limelight3A.getLatestResult();
//
//            if (limelight_result != null) {
//                // Access general information
//                Pose3D botpose = limelight_result.getBotpose();
//                double captureLatency = limelight_result.getCaptureLatency();
//                double targetingLatency = limelight_result.getTargetingLatency();
//                double parseLatency = limelight_result.getParseLatency();
//                telemetry.addData("LL Latency", captureLatency + targetingLatency);
//                telemetry.addData("Parse Latency", parseLatency);
//                telemetry.addData("PythonOutput", java.util.Arrays.toString(limelight_result.getPythonOutput()));
//
//                if (limelight_result.isValid()) {
//                    telemetry.addData("tx", limelight_result.getTx());
//                    telemetry.addData("txnc", limelight_result.getTxNC());
//                    telemetry.addData("ty", limelight_result.getTy());
//                    telemetry.addData("tync", limelight_result.getTyNC());
//                    telemetry.addData("Botpose", botpose.toString());
//
//                    // Access color results
//                    List<LLResultTypes.ColorResult> colorResults = limelight_result.getColorResults();
//                    LLResultTypes.ColorResult cr = colorResults.get(0);
//
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//
////                    crosshair_x = 8 * Math.tan(cr.getTargetXDegrees());
//                    crosshair_x = cr.getTargetXDegrees() * ANGLE_TO_DISTANCE_FACTOR;
//                    crosshair_y = cr.getTargetYDegrees() * ANGLE_TO_DISTANCE_FACTOR;
//                    crosshair_angle = 0;
//
//                    telemetry.addData("Crosshair", "X: %.2f, Y: %.2f", crosshair_x, crosshair_y);
//                }
//            } else {
//                telemetry.addData("Limelight", "No data available");            }
//
//            telemetry.update();
//            limelight3A.stop();
//        }
//   }

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
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Elbow elbow = new Bot_Elbow(hardwareMap);
        Bot_Gripper gripper = new Bot_Gripper(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        Bot_Drivebase drivebase = new Bot_Drivebase(hardwareMap);
        FTCTelemetry ftctelemetry = new FTCTelemetry(hardwareMap);
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
                                indicatorlight.TurnIndicatorLight_Off(),
                                flag.FlagDown(),
                                elbow.ElbowCollect()
                        ),
                        new SleepAction(0.25),  // wait for the elbow to turn

                        // Score preloaded sample to high basket
                        new ParallelAction(
                                bucket.BucketCatch(),
                                slides.SlidesUpHigh(),
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
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition1,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
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
                                        slides.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 2nd sample from mat
                        new ParallelAction(
                                arm.ArmPrepareToCollect(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                headlight.headlight_On(),
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition2,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
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
                                        slides.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 3rd sample from mat
                        new ParallelAction(
                                arm.ArmPrepareToCollect(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                headlight.headlight_On(),
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition3,
                                elbow.ElbowCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToTarget_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
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
                                        slides.SlidesUpHigh()
                                )
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to Level 1 Ascend
                        new ParallelAction(
                            actDriveToPark,
                            indicatorlight.TurnIndicatorLight_Off(),
                            bucket.BucketCatch(),
                            flag.FlagScore(),
                            new SequentialAction(
                                    elbow.ElbowCollect(),
                                    gripper.GripperIn(),
                                    new SleepAction(0.2),
                                    arm.ArmCollapsedIntoRobot(),
                                    slides.SlidesDownGround()
                            )
                        ),
                        ftctelemetry.UpdateTotalDuration()
                )
        );
    }
}