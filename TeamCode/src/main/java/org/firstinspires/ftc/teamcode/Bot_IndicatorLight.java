package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_IndicatorLight {
    final double INDICATOR_LIGHT_OFF = 0;
    final double INDICATOR_LIGHT_GREEN = 0.5;
    final double INDICATOR_LIGHT_RED = 0.279;
    final double INDICATOR_LIGHT_BLUE = 0.611;

    private String desiredColor;

    private ServoImplEx indicatorlight;

    public Bot_IndicatorLight(HardwareMap hardwareMap) {
        indicatorlight = hardwareMap.get(ServoImplEx.class, "indicator_light");
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

    public class TurnIndicatorLight_AllianceColor implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (desiredColor.equals("RED")) {
                indicatorlight.setPosition(INDICATOR_LIGHT_RED);
            }
            else if (desiredColor.equals("BLUE")) {
                indicatorlight.setPosition(INDICATOR_LIGHT_BLUE);
            }
            return false;
        }
    }

    public Action TurnIndicatorLight_AllianceColor(String color) {
        desiredColor = color;
        return new TurnIndicatorLight_AllianceColor();
    }
}