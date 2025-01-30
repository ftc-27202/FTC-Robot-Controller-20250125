package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_Flag {
    // For physical install, 1.0 = Flag is facing all the way down
    final double FLAG_DOWN = 1.0;
    final double FLAG_SCORE = 0.30;

    private Servo flag;

    public Bot_Flag(HardwareMap hardwareMap) {
        flag = hardwareMap.get(Servo.class, "flag");
    }

    public class FlagDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flag.setPosition(FLAG_DOWN);
            return false;
        }
    }

    public Action FlagDown() {
        return new FlagDown();
    }

    public class FlagScore implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            flag.setPosition(FLAG_SCORE);
            return false;
        }
    }

    public Action FlagScore() {
        return new FlagScore();
    }
}