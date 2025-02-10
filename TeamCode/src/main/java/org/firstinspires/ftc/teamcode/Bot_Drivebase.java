package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_Drivebase {
    private Bot_Camera camera;
    private Vector2d crosshair = new Vector2d(0,0);
    private HardwareMap local_hardwareMap;
    private String allianceColor;
    private String orientation;
    public LLResult LLResult;
    public LLStatus LLStatus;

    public Bot_Drivebase(HardwareMap hardwareMap, String InputAllianceColor) {
        local_hardwareMap = hardwareMap;
        camera = new Bot_Camera(hardwareMap);
        allianceColor = InputAllianceColor;

        LLResult = camera.LimelightResult;
        LLStatus = camera.LimelightStatus;
    }
    public LLResult getCameraResult(){
        return camera.limelight3A.getLatestResult();
    }
    public LLStatus getCameraStatus(){
        return camera.limelight3A.getStatus();
    }
    public class AlignToNeutralSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            crosshair = new Vector2d(0, 0);
            crosshair = camera.ObtainCrosshair("YELLOW", orientation);
            packet.put("crosshair.x", crosshair.x);
            packet.put("crosshair.y", crosshair.y);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(crosshair.y, -crosshair.x))
                            .build()
            );

            return false;
        }
    }

    public Action AlignToNeutralSample(String inOrientation) {
        orientation = inOrientation;
        return new AlignToNeutralSample();}

    public class AlignToAllianceSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            crosshair = new Vector2d(0, 0);
            crosshair = camera.ObtainCrosshair(allianceColor, orientation);
            packet.put("crosshair.x", crosshair.x);
            packet.put("crosshair.y", crosshair.y);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(crosshair.y, -crosshair.x))
                            .build()
            );

            return false;
        }
    }

    public Action AlignToAllianceSample(String inOrientation) {
        orientation = inOrientation;
        return new AlignToAllianceSample();}

    public class AlignToSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            crosshair = new Vector2d(0, 0);
            crosshair = camera.ObtainCrosshair(allianceColor, "VERTICAL_SPECIMEN");
            packet.put("color", allianceColor);
            packet.put("crosshair.x", crosshair.x);
            packet.put("crosshair.y", crosshair.y);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(crosshair.y, -(crosshair.x)))
                            .build()
            );

            return false;
        }
    }

    public Action AlignToSpecimen() {
        return new AlignToSpecimen();}

    public class MoveBackToToInitialPose_ForSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, new Pose2d(0, 0, 0));

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-crosshair.y, crosshair.x))
                            .build()
            );
            return false;
        }
    }

    public Action MoveBackToToInitialPose_ForSample() {
        return new MoveBackToToInitialPose_ForSample();
    }

    public class MoveBackToToInitialPose_ForSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, new Pose2d(0, 0, 0));

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-crosshair.y, crosshair.x))
                            .build()
            );
            return false;
        }
    }

    public Action MoveBackToToInitialPose_ForSpecimen() {
        return new MoveBackToToInitialPose_ForSpecimen();
    }

    public class MoveBackForSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-6, 0))
                            .build()
            );

            return false;
        }
    }

    public Action MoveBackForSpecimen() {
        return new MoveBackForSpecimen();}

    public class MoveForwardForSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(10, 0))
                            .build()
            );

            return false;
        }
    }

    public Action MoveForwardForSpecimen() {
        return new MoveForwardForSpecimen();}
}
