package org.firstinspires.ftc.teamcode;

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

    private Pose2d initialPose = new Pose2d(0, 0, 0);
    private MecanumDrive bot;
    private Bot_Camera camera;
    private Vector2d crosshair = new Vector2d(0,0);

    public Bot_Drivebase(HardwareMap hardwareMap) {
        bot = new MecanumDrive(hardwareMap, initialPose);
        camera = new Bot_Camera(hardwareMap);
    }

    public class AlignToTarget_X implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double distance_to_target_x;
            double distance_to_target_y;

            crosshair = camera.ObtainCrosshair();
            distance_to_target_x = 0;
            distance_to_target_y = -crosshair.x;

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
            double distance_to_target_x;
            double distance_to_target_y;

            distance_to_target_x = 0;
            distance_to_target_y = crosshair.x;

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