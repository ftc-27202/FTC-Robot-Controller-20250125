package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_Elbow {
    final double ELBOW_COLLECT = 0.33;
    final double ELBOW_DEPOSIT = 1.0;

    private Servo elbow;

    public Bot_Elbow(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(Servo.class, "elbow");
    }

    public class ElbowDeposit implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(ELBOW_DEPOSIT);
            return false;
        }
    }

    public Action ElbowDeposit() {
        return new ElbowDeposit();
    }

    public class ElbowCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(ELBOW_COLLECT);
            return false;
        }
    }

    public Action ElbowCollect() {
        return new ElbowCollect();
    }
}