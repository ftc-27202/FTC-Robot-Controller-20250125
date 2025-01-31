package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_Drivebase {
    private Bot_Camera camera;
    private Vector2d crosshair = new Vector2d(0,0);
    private HardwareMap local_hardwareMap;
    private String allianceColor;

    public Bot_Drivebase(HardwareMap hardwareMap, String InputAllianceColor) {
        local_hardwareMap = hardwareMap;
        camera = new Bot_Camera(hardwareMap);
        allianceColor = InputAllianceColor;
    }

    public class AlignToNeutralSample_X implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            crosshair = camera.ObtainCrosshair("YELLOW");
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

    public Action AlignToNeutralSample_X() {
        return new AlignToNeutralSample_X();}

    public class AlignToAllianceElement_X implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            Pose2d initialPose = new Pose2d(0, 0, 0);
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, initialPose);

            crosshair = camera.ObtainCrosshair(allianceColor);
            packet.put("color", allianceColor);
            packet.put("crosshair.x", crosshair.x);
            packet.put("crosshair.y", crosshair.y);

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(-crosshair.y, -crosshair.x))

                            .build()
            );

            return false;
        }
    }

    public Action AlignToAllianceElement_X() {
        return new AlignToAllianceElement_X();}

    public class MoveBackToToInitialPose_X implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            MecanumDrive bot = new MecanumDrive(local_hardwareMap, new Pose2d(0, 0, 0));

            Actions.runBlocking(
                    bot.actionBuilder(new Pose2d(0, 0, 0))
                            .strafeTo(new Vector2d(crosshair.y, crosshair.x))
                            .build()
            );
            return false;
        }
    }

    public Action MoveBackToToInitialPose_X() {
        return new MoveBackToToInitialPose_X();
    }
}