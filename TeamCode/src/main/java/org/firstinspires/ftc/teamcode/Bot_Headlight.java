package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Headlight {
    final double HEADLIGHT_OFF = 0;
    final double HEADLIGHT_ON = 0.5;

    private ServoImplEx headlight;

    public Bot_Headlight(HardwareMap hardwareMap) {
        headlight = hardwareMap.get(ServoImplEx.class, "headlight");
    }

    public class headlight_Off implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            headlight.setPosition(HEADLIGHT_OFF);
            return false;
        }
    }

    public Action headlight_Off() {
        return new headlight_Off();
    }

    public class headlight_On implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            headlight.setPosition(HEADLIGHT_ON);
            return false;
        }
    }

    public Action headlight_On() {
        return new headlight_On();
    }
}