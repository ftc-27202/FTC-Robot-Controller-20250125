package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_IndicatorLight {
    final double INDICATOR_LIGHT_OFF = 0;
    final double INDICATOR_LIGHT_GREEN = 0.5;

    private Servo indicatorlight;

    public Bot_IndicatorLight(HardwareMap hardwareMap) {
        indicatorlight = hardwareMap.get(Servo.class, "indicator_light");
    }

    public class TurnIndicatorLight_Off implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            indicatorlight.setPosition(INDICATOR_LIGHT_OFF);
            return false;
        }
    }

    public Action TurnIndicatorLight_Off() {
        return new TurnIndicatorLight_Off();
    }

    public class TurnIndicatorLight_Green implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            indicatorlight.setPosition(INDICATOR_LIGHT_GREEN);
            return false;
        }
    }

    public Action TurnIndicatorLight_Green() {
        return new TurnIndicatorLight_Green();
    }

}