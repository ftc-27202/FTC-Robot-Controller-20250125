package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class jeff_base_auto_basket extends LinearOpMode {
    private String allianceColor;

    public void setAllianceColor(String inAllianceColor) {
        allianceColor = inAllianceColor;
    }
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
        int HighBasketHeading = 45;
        Vector2d HighBasketVector = new Vector2d(-55, -48);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryActionBuilder trajDriveToHighBasket = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-44, -60));

        TrajectoryActionBuilder trajDriveToCollectSamplePosition1 = trajDriveToHighBasket.endTrajectory().fresh()
                .strafeToSplineHeading(new Vector2d(-51, -45), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket2 = trajDriveToCollectSamplePosition1.endTrajectory().fresh()
                .turnTo(Math.toRadians(HighBasketHeading))
                .strafeToConstantHeading(HighBasketVector);

        TrajectoryActionBuilder trajDriveToCollectSamplePosition2 = trajDriveToHighBasket2.endTrajectory().fresh()
                .turnTo(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-61, -45), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket3 = trajDriveToCollectSamplePosition2.endTrajectory().fresh()
                .turnTo(Math.toRadians(HighBasketHeading))
                .strafeToConstantHeading(HighBasketVector);

        TrajectoryActionBuilder trajDriveToCollectSamplePosition3 = trajDriveToHighBasket3.endTrajectory().fresh()
                .turnTo(Math.toRadians(120))
                .splineToConstantHeading(new Vector2d(-61, -43), Math.toRadians(90));

        TrajectoryActionBuilder trajDriveToHighBasket4 = trajDriveToCollectSamplePosition3.endTrajectory().fresh()
                .turnTo(Math.toRadians(HighBasketHeading))
                .strafeToConstantHeading(HighBasketVector);

        TrajectoryActionBuilder trajDriveToPark = trajDriveToHighBasket4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(-36, -10, Math.toRadians(0)), 0)
                .lineToXConstantHeading(-27);

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
                                indicatorlight.TurnIndicatorLight_AllianceColor(allianceColor),
                                flag.FlagDown(),
                                bucket.BucketCatch()
                        ),

                        // Score preloaded sample to high basket
                        new ParallelAction(
                                bucket.BucketCatch(),
                                slides.SlidesUpHigh(),
                                actDriveToHighBasket
                        ),
                        bucket.BucketDump(),
                        new SleepAction(0.4),

                        // Drive to collect 1st sample from mat
                        new ParallelAction(
                                new SequentialAction(
                                        wrist.WristCollect(),
                                        new SleepAction(0.5),
                                        arm.ArmPrepareToCollect()
                                ),
                                headlight.headlight_On(),
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition1,
                                wrist.WristCollect(),
                                gripper.GripperOut()
                        ),
                        drivebase.AlignToNeutralSample("VERTICAL"),
                        arm.ArmCollectSample(),
                        gripper.GripperIn(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket2,
                                headlight.headlight_Off(),
                                bucket.BucketCatch(),
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
                                headlight.headlight_On(),
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition2,
                                wrist.WristCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        )
                        ,
                        drivebase.AlignToNeutralSample("VERTICAL"),
                        arm.ArmCollectSample(),
                        gripper.GripperIn(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket3,
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
                                headlight.headlight_On(),
                                slides.SlidesDownCatch(),
                                actDriveToCollectSamplePosition3,
                                wrist.WristCollect(),
                                gripper.GripperOut(),
                                bucket.BucketCatch()
                        ),
                        drivebase.AlignToNeutralSample("VERTICAL"),
                        arm.ArmCollectSample(),
                        gripper.GripperIn(),
                        new SleepAction(0.2),
                        drivebase.MoveBackToToInitialPose_ForSample(),
                        new ParallelAction(
                                actDriveToHighBasket4,
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
                            bucket.BucketCatch(),
                            flag.FlagScore(),
                            new SequentialAction(
                                    wrist.WristCollect(),
                                    gripper.GripperIn(),
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