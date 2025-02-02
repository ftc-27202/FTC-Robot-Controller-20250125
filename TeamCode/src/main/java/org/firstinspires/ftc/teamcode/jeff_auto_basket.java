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

@Autonomous(name = "01 Basket Side", group = "Autonomous")
public class jeff_auto_basket extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-41, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Wrist wrist = new Bot_Wrist(hardwareMap);
        Bot_Gripper gripper = new Bot_Gripper(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        Bot_Drivebase drivebase = new Bot_Drivebase(hardwareMap, "NEUTRAL");
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
                        new ParallelAction(
                                headlight.headlight_Off(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                flag.FlagDown(),
                                wrist.WristCollect()
                        ),
                        new SleepAction(0.25),  // wait for the elbow to turn

                        // Score preloaded sample to high basket
                        new ParallelAction(
                                bucket.BucketCatch(),
                                slides.SlidesUpHigh(),
                                actDriveToHighBasket,
                                wrist.WristCollect()
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
                                wrist.WristCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToNeutralSample(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket2,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                wrist.WristDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        new SequentialAction(
                                            gripper.GripperOut(),
                                            new SleepAction(0.2)
                                        ),
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
                                wrist.WristCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToNeutralSample(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket3,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                wrist.WristDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        new SequentialAction(
                                                gripper.GripperOut(),
                                                new SleepAction(0.2)
                                        ),
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
                                wrist.WristCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToNeutralSample(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket4,
                                indicatorlight.TurnIndicatorLight_Green(),
                                headlight.headlight_Off(),
                                new SequentialAction(
                                        new ParallelAction(
                                                wrist.WristDeposit(),
                                                arm.ArmDeposit()
                                        ),
                                        new SequentialAction(
                                                gripper.GripperOut(),
                                                new SleepAction(0.2)
                                        ),
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
                                    wrist.WristCollect(),
                                    gripper.GripperGrabInwards(),
                                    new SleepAction(0.2),
                                    arm.ArmCollapsedIntoRobot(),
                                    slides.SlidesDownGround()
                            )
                        )
                )
        );
        telemetry.addData("Duration", this.getRuntime());
        telemetry.update();
    }
}