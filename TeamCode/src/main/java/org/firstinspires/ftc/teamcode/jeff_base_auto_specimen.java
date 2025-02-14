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
        Pose2d initialPose = new Pose2d(11, -60, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Bot_Slides slides = new Bot_Slides(hardwareMap);
        Bot_Bucket bucket = new Bot_Bucket(hardwareMap);
        Bot_Arm arm = new Bot_Arm(hardwareMap);
        Bot_Wrist wrist = new Bot_Wrist(hardwareMap);
        Bot_Claw claw = new Bot_Claw(hardwareMap);
        Bot_WristRotation wristRotation = new Bot_WristRotation(hardwareMap);
        Bot_Flag flag = new Bot_Flag(hardwareMap);
        Bot_Headlight headlight = new Bot_Headlight(hardwareMap);
        Bot_IndicatorLight indicatorlight = new Bot_IndicatorLight(hardwareMap);
        Bot_Drivebase drivebase = new Bot_Drivebase(hardwareMap, allianceColor);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToSubmersible1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(-2, -26), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToSample1 = trajDriveToSubmersible1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(52, -42), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToDropSample1 = trajDriveToSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(48, -40), Math.toRadians(-60));

        TrajectoryActionBuilder trajDriveToSpecimen2 = trajDriveToDropSample1.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(26, -54), Math.toRadians(0))
                .strafeToSplineHeading(new Vector2d(30, -54), Math.toRadians(0));

        TrajectoryActionBuilder trajDriveToSubmersible2 = trajDriveToSpecimen2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(2, -28), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToSpecimen3 = trajDriveToSubmersible2.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(30, -54), Math.toRadians(0));

        TrajectoryActionBuilder trajDriveToSubmersible3 = trajDriveToSpecimen3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(6, -28), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToPark = trajDriveToSubmersible3.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(60, -50), Math.toRadians(90));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        Action actDriveToSubmersible1 = trajDriveToSubmersible1.build();
        Action actDriveToSample1 = trajDriveToSample1.build();
        Action actDriveToDropSample1 = trajDriveToDropSample1.build();
        Action actDriveToSpecimen2 = trajDriveToSpecimen2.build();
        Action actDriveToSubmersible2 = trajDriveToSubmersible2.build();
        Action actDriveToSpecimen3 = trajDriveToSpecimen3.build();
        Action actDriveToSubmersible3 = trajDriveToSubmersible3.build();
        Action actDriveToPark = trajDriveToPark.build();

        waitForStart();

        this.resetRuntime();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                headlight.headlight_Off(),
                                indicatorlight.TurnIndicatorLight_AllianceColor(allianceColor),
                                flag.FlagDown(),
                                new SequentialAction(
                                    wrist.WristCollect(),
                                    new SleepAction(0.30)),
                                bucket.BucketDump(),
                                wristRotation.wristRotationSpecimen(),
                                claw.ClawCloseSpecimenToScore()
                        ),

                        // Score Preloaded Specimen: Before
                        new ParallelAction(
                                actDriveToSubmersible1,
                                new SequentialAction(
                                    slides.SlidesClearArm(),
                                    wristRotation.wristRotationSpecimen(),
                                    arm.ArmDownSpecimenBeforeScore(),
                                    slides.SlidesDownGround()
                                )
                        ),

                        // Score Preloaded Specimen: After
                        new SequentialAction(
                                claw.ClawCloseSpecimenToSlide(),
                                new ParallelAction(
                                        arm.ArmSpecimenAfterScore(),
                                        drivebase.MoveBackForSpecimen()),
                                new SleepAction(0.30),
                                claw.ClawOpen(),
                                wristRotation.wristRotationVertical(),
                                arm.ArmUpSpecimenBeforeScore()
                        ),

                        // Drive to collect sample 1 from mat
                        new ParallelAction(
                                claw.ClawOpen(),
                                headlight.headlight_On(),
                                wrist.WristCollect(),
                                new SequentialAction(
                                        new ParallelAction(
                                            actDriveToSample1,
                                            new SequentialAction(
                                                new SleepAction(1.5),
                                                arm.ArmPrepareToCollectSpecimenAuto())),
                                        drivebase.AlignToAllianceSample("VERTICAL"),
                                        arm.ArmCollectSample(),
                                        claw.ClawClose(),
                                        new SleepAction(0.2),
                                        drivebase.MoveBackToToInitialPose_ForSample()
                                )
                        ),

                        // Drive to drop sample 1 to observation zone
                        new SequentialAction(
                        bucket.BucketInitial(),
                                arm.ArmCollected(),
                                actDriveToDropSample1,
                                claw.ClawOpen(),
                                new SleepAction(0.2)),

                        // Drive to collect specimen 2 from mat
                        new SequentialAction(
                                new ParallelAction(
                                    actDriveToSpecimen2,
                                    arm.ArmPrepareToCollectSpecimenAuto(),
                                    wrist.WristCollect()),
                                drivebase.AlignToSpecimen(),
                                arm.ArmCollectSample(),
                                claw.ClawClose(),
                                new SleepAction(0.2),
                                drivebase.MoveBackToToInitialPose_ForSpecimen()),

                        // Score specimen 2: Before
                        new ParallelAction(
                                actDriveToSubmersible2,
                                claw.ClawCloseSpecimenToScore(),
                                wristRotation.wristRotationSpecimen(),
                                arm.ArmUpSpecimenBeforeScore()),

                        // Score specimen 2: After
                        new SequentialAction(
                                claw.ClawCloseSpecimenToSlide(),
                                new ParallelAction(
                                        arm.ArmSpecimenAfterScore(),
                                        drivebase.MoveBackForSpecimen()),
                                new SleepAction(0.30),
                                claw.ClawOpen(),
                                arm.ArmUpSpecimenBeforeScore()
                        ),

                        // Drive to collect specimen 3 from mat
                        new SequentialAction(
                                new ParallelAction(
                                        actDriveToSpecimen3,
                                        new SequentialAction(
                                                new SleepAction(1.5),
                                                arm.ArmPrepareToCollectSpecimenAuto()),
                                        wristRotation.wristRotationVertical(),
                                        wrist.WristCollect(),
                                        claw.ClawOpen()),
                                drivebase.AlignToSpecimen(),
                                arm.ArmCollectSample(),
                                claw.ClawClose(),
                                new SleepAction(0.2),
                                drivebase.MoveBackToToInitialPose_ForSpecimen()
                        ),

                        // Score specimen 3: Before
                        new ParallelAction(
                                actDriveToSubmersible3,
                                claw.ClawCloseSpecimenToScore(),
                                wristRotation.wristRotationSpecimen(),
                                arm.ArmUpSpecimenBeforeScore()),

                        // Score specimen 3: After
                        new SequentialAction(
                                claw.ClawCloseSpecimenToSlide(),
                                new ParallelAction(
                                        arm.ArmSpecimenAfterScore(),
                                        drivebase.MoveBackForSpecimen()),
                                new SleepAction(0.30),
                                claw.ClawOpen(),
                                arm.ArmUpSpecimenBeforeScore()
                        ),

                        // Park and Robot to Initial Position
                        new ParallelAction(
                                actDriveToPark,
                                wristRotation.wristRotationVertical(),
                                new SequentialAction(
                                        slides.SlidesClearArmAutoSpecimen(),
                                        bucket.BucketCatch(),
                                        arm.ArmCollapsedIntoRobot(),
                                        slides.SlidesDownGround()),
                                headlight.headlight_Off()
                        )
                )
        );
        telemetry.addData("Duration", this.getRuntime());
        telemetry.update();
    }
}