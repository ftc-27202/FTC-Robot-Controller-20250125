package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_WristRotation {
    // For physical install,
    final double WRIST_ROTATION_VERTICAL_ALIGNMENT = 1.0;
    final double WRIST_ROTATION_HORIZONTAL_ALIGNMENT = 0.74;
    final double WRIST_ROTATION_SPECIMEN = 0.20;
    final double WRIST_ROTATION_AUTO_SAMPLE_3 = 0.85;

//    before Feb 14, 2025
//    final double WRIST_ROTATION_VERTICAL_ALIGNMENT = 0.0;
//    final double WRIST_ROTATION_HORIZONTAL_ALIGNMENT = 0.5;
//    final double WRIST_ROTATION_SPECIMEN = 1.0;
//    final double WRIST_ROTATION_AUTO_SAMPLE_3 = 0.90;

    private double inTriggerValue = 0.0;
    private double posRotation = WRIST_ROTATION_HORIZONTAL_ALIGNMENT;
    private double posPriorRotation = posRotation;

    private ServoImplEx wristRotation;

    public Bot_WristRotation(HardwareMap hardwareMap) {
        wristRotation = hardwareMap.get(ServoImplEx.class, "wristRotation");
    }

    public class wristRotationVertical implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristRotation.setPosition(WRIST_ROTATION_VERTICAL_ALIGNMENT);
            posRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;
            posPriorRotation = posRotation;
            packet.put("wristRotationPos", wristRotation.getPosition());
            return false;
        }
    }

    public Action wristRotationVertical() {
        return new wristRotationVertical();
    }

    public class wristRotationHorizontal implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristRotation.setPosition(WRIST_ROTATION_HORIZONTAL_ALIGNMENT);
            posRotation = WRIST_ROTATION_HORIZONTAL_ALIGNMENT;
            posPriorRotation = posRotation;
            packet.put("wristRotationPos", wristRotation.getPosition());
            return false;
        }
    }

    public Action wristRotationHorizontal() {
        return new wristRotationHorizontal();
    }

    public class wristRotationAutoSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristRotation.setPosition(WRIST_ROTATION_AUTO_SAMPLE_3);
            packet.put("wristRotationPos", wristRotation.getPosition());
            return false;
        }
    }

    public Action wristRotationAutoSample() {
        return new wristRotationAutoSample();
    }


    public class wristRotationSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wristRotation.setPosition(WRIST_ROTATION_SPECIMEN);
            packet.put("wristRotationPos", wristRotation.getPosition());
            return false;
        }
    }

    public Action wristRotationSpecimen() {
        return new wristRotationSpecimen();
    }

    public class wristRotateManually implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (posPriorRotation == WRIST_ROTATION_VERTICAL_ALIGNMENT) {
                posRotation -= inTriggerValue * 0.0017;
            } else {
                posRotation += inTriggerValue * 0.0017;
            };
            if (posRotation > WRIST_ROTATION_VERTICAL_ALIGNMENT) {
                posRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;
                posPriorRotation = posRotation;
            };
            if (posRotation < WRIST_ROTATION_SPECIMEN){
                posRotation = WRIST_ROTATION_SPECIMEN;
                posPriorRotation = posRotation;
            };

            wristRotation.setPosition(posRotation);
            packet.put("wristRotationPos", wristRotation.getPosition());
            return false;
        }
    }

    public Action wristRotateManually(double inputTriggerValue) {
        inTriggerValue = inputTriggerValue;
        return new wristRotateManually();
    }
}