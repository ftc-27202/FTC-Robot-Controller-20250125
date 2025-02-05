package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class jeff_base_auto_specimen extends LinearOpMode {
    private String allianceColor;

    public void setAllianceColor(String inAllianceColor) {
        allianceColor = inAllianceColor;
    }

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
                .strafeToSplineHeading(new Vector2d(-6, -26), Math.toRadians(90));

//        TrajectoryActionBuilder trajDriveBackToScoreSpecimen1 = trajDriveToSubmersible1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(-6, -33), new TranslationalVelConstraint(20.0));
//
        TrajectoryActionBuilder trajDriveToSample1 = trajDriveToSubmersible1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(34, -46));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Action actDriveToSubmersible1 = trajDriveToSubmersible1.build();
//        Action actDriveBackToScoreSpecimen1 = trajDriveBackToScoreSpecimen1.build();
        Action actDriveToSample1 = trajDriveToSample1.build();

        waitForStart();

        this.resetRuntime();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                headlight.headlight_Off(),
                                indicatorlight.TurnIndicatorLight_AllianceColor(allianceColor),
                                flag.FlagDown(),
                                wrist.WristCollect(),
                                new SleepAction(0.2),
                                bucket.BucketDump(),
                                gripper.GripperIn()
                        ),
                        // Score Preloaded Specimen
                        new ParallelAction(
                                actDriveToSubmersible1,
                                new SequentialAction(
                                    slides.SlidesUpAscend(),
                                    arm.ArmDownSpecimenBeforeScore(),
                                    slides.SlidesDownGround(),
                                    bucket.BucketCatch()
                                )
                        ),
                        new SequentialAction(
                                new ParallelAction(
                                        arm.ArmSpecimenAfterScore(),
                                        drivebase.MoveBackForSpecimen()),
                                new SleepAction(0.20),
                                gripper.GripperOut(),
                                arm.ArmUpSpecimenBeforeScore(),
                                drivebase.MoveForwardForSpecimen(),
                                bucket.BucketOff()
                        )
//                        new SequentialAction(
//                                new ParallelAction(
//                                        actDriveBackToScoreSpecimen1,
//                                        drivebase.MoveBackForSpecimen()),
//                                new SleepAction(0.20),
//                                gripper.GripperOut(),
//                                new SleepAction(0.50)),
//                                drivebase.MoveForwardForSpecimen())
//                        ,

//                        // Drive to collect sample 1 from mat
//                        new ParallelAction(
//                                headlight.headlight_On(),
//                                arm.ArmPrepareToCollect(),
//                                actDriveToSample1,
//                                wrist.WristCollect(),
//                                gripper.GripperOut()
//                        )
//                        ,
//                        drivebase.AlignToAllianceSample("VERTICAL"),
//                        arm.ArmCollectSample(),
//                        gripper.GripperIn(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_ForSpecimen(),
//                        headlight.headlight_Off(),

//                        // Drop Sample to Observation Zone
//                        arm.ArmDropSampleToZone(),
//                        wrist.WristDeposit(),
//                        new SleepAction(0.5),  // wait for the elbow to turn
//                        gripper.GripperOut(),
//                        new SleepAction(0.25),    // wait for sample to drop to zone
//
//                        // Drive to collect sample 2 from mat
//                        new ParallelAction(
//                                headlight.headlight_On(),
//                                arm.ArmPrepareToCollect(),
//                                actDriveToSample2,
//                                wrist.WristCollect(),
//                                gripper.GripperOut()
//                        ),
//                        drivebase.AlignToAllianceSample("VERTICAL"),
//                        arm.ArmCollectSample(),
//                        gripper.GripperIn(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_ForSpecimen(),
//                        headlight.headlight_Off(),
//
//                        // Drop Sample to Observation Zone
//                        arm.ArmDropSampleToZone(),
//                        wrist.WristDeposit(),
//                        new SleepAction(0.5),  // wait for the elbow to turn
//                        gripper.GripperOut(),
//                        new SleepAction(0.25),    // wait for sample to drop to zone
//
//                        // Drive to collect sample 3 from mat
//                        new ParallelAction(
//                                headlight.headlight_On(),
//                                arm.ArmPrepareToCollect(),
//                                actDriveToSample3,
//                                wrist.WristCollect(),
//                                gripper.GripperOut()
//                        ),
//                        drivebase.AlignToAllianceSample("VERTICAL"),
//                        arm.ArmCollectSample(),
//                        gripper.GripperIn(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_ForSpecimen(),
//                        headlight.headlight_Off(),
//
//                        // Drop Sample to Observation Zone
//                        actDriveToDropSample3,
//                        arm.ArmDropSampleToZone(),
//                        wrist.WristDeposit(),
//                        new SleepAction(0.5),  // wait for the elbow to turn
//                        gripper.GripperOut(),
//                        new SleepAction(0.25),    // wait for sample to drop to zone
//
//                        // Drive to collect specimen 2 from mat
//                        new ParallelAction(
//                                headlight.headlight_On(),
//                                actDriveToCollectSpecimen2,
//                                arm.ArmPrepareToCollect(),
//                                wrist.WristCollect(),
//                                gripper.GripperOut()
//                        ),
//                        drivebase.AlignToSpecimen(),
//                        arm.ArmCollectSpecimen(),
//                        gripper.GripperIn(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_ForSpecimen(),
//                        headlight.headlight_Off()
//                        ,

//                        new ParallelAction(
//                                actDriveToSubmersible2,
//                                arm.ArmUpSpecimenBeforeScore()
//                        ),
//                        new ParallelAction(
//                                actDriveBackToScoreSpecimen2,
//                                arm.ArmSpecimenAfterScore()
//                        ),
//
//                        // Drive to collect specimen 3 from mat
//                        new ParallelAction(
//                                headlight.headlight_On(),
//                                actDriveToCollectSpecimen3,
//                                arm.ArmPrepareToCollect(),
//                                wrist.WristCollect(),
//                                gripper.GripperOut()
//                        ),
//                        drivebase.AlignToSpecimen(),
//                        arm.ArmCollectSpecimen(),
//                        gripper.GripperIn(),
//                        new SleepAction(0.2),
//                        drivebase.MoveBackToToInitialPose_ForSpecimen(),
//                        headlight.headlight_Off(),
//
//                        new ParallelAction(
//                                actDriveToSubmersible3,
//                                arm.ArmUpSpecimenBeforeScore()
//                        ),
//                        new ParallelAction(
//                                actDriveBackToScoreSpecimen3,
//                                arm.ArmSpecimenAfterScore()
//                        ),
//
//                        new SleepAction(5)  //temporary
                )
        );
    }
}