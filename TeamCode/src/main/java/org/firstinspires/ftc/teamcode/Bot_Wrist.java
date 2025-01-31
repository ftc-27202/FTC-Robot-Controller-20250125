package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Wrist {
    final double WRIST_COLLECT = 0.33;
    final double WRIST_DEPOSIT = 1.0;

    private ServoImplEx wrist;

    public Bot_Wrist(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(ServoImplEx.class, "wrist");
    }

    public class WristDeposit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_DEPOSIT);
            return false;
        }
    }

    public Action WristDeposit() {
        return new WristDeposit();
    }

    public class WristCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wrist.setPosition(WRIST_COLLECT);
            return false;
        }
    }

    public Action WristCollect() {
        return new WristCollect();
    }
}