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

@Autonomous(name = "02_Auto (Red Specimen)", group = "Autonomous")
public class jeff_auto_red_specimen extends LinearOpMode {

    final String allianceColor = "RED";  // Valid Values: RED or BLUE
//    final int allianceColor = "BLUE";  // Valid Values: RED or BLUE

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(6, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Wrist wrist = new Bot_Wrist(hardwareMap);
        Bot_Gripper gripper = new Bot_Gripper(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        Bot_Drivebase drivebase = new Bot_Drivebase(hardwareMap, allianceColor);
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
                        new ParallelAction(
                                headlight.headlight_Off(),
                                indicatorlight.TurnIndicatorLight_Off(),
                                flag.FlagDown(),
                                wrist.WristCollect(),
                                bucket.BucketDump(),
                                gripper.GripperGrabInwards()
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
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToAllianceElement_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        arm.ArmDropSampleToZone(),
                        wrist.WristDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect sample 2 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                arm.ArmPrepareToCollect(),
                                actDriveToSample2,
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToAllianceElement_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        arm.ArmDropSampleToZone(),
                        wrist.WristDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect sample 3 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                arm.ArmPrepareToCollect(),
                                actDriveToSample3,
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToAllianceElement_X(),
                        arm.ArmCollectSample(),
                        gripper.GripperGrabInwards(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_X(),
                        headlight.headlight_Off(),

                        // Drop Sample to Observation Zone
                        actDriveToDropSample3,
                        arm.ArmDropSampleToZone(),
                        wrist.WristDeposit(),
                        new SleepAction(0.5),  // wait for the elbow to turn
                        gripper.GripperOut(),
                        new SleepAction(0.25),    // wait for sample to drop to zone

                        // Drive to collect specimen 2 from mat
                        new ParallelAction(
                                headlight.headlight_On(),
                                actDriveToCollectSpecimen2,
                                arm.ArmPrepareToCollect(),
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToAllianceElement_X(),
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
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToAllianceElement_X(),
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

                        new SleepAction(5)  //temporary
                )
        );
    }
}