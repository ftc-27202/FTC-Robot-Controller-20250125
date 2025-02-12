package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Flag {
    // For physical install, 1.0 = Flag is facing all the way down
    final double FLAG_DOWN = 1.0;
    final double FLAG_SCORE = 0.20;

    private ServoImplEx flag;

    public Bot_Flag(HardwareMap hardwareMap) {
        flag = hardwareMap.get(ServoImplEx.class, "flag");
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

    public class FlagToggle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (flag.getPosition() >= 0.50) {
                flag.setPosition(FLAG_SCORE);
            }
            else flag.setPosition(FLAG_DOWN);
            return false;
        }
    }

    public Action FlagToggle() {
        return new FlagToggle();
    }
}