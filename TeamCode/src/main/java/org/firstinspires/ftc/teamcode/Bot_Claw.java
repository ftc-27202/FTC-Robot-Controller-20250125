package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Claw {
    // For physical install, ???
    public final double CLAW_OPEN = 0.42;
    public final double CLAW_CLOSE= 0.02;
    public final double CLAW_CLOSE_FOR_SPECIMEN = 0.06;

    private ServoImplEx claw;

    public Bot_Claw(HardwareMap hardwareMap) {
        claw = hardwareMap.get(ServoImplEx.class, "gripper");
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(CLAW_CLOSE);
            packet.put("ClawPos", claw.getPosition());
            return false;
        }
    }

    public Action ClawClose() {
        return new ClawClose();
    }

    public class ClawCloseSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(CLAW_CLOSE_FOR_SPECIMEN);
            packet.put("ClawPos", claw.getPosition());
            return false;
        }
    }

    public Action ClawCloseSpecimen() {
        return new ClawCloseSpecimen();
    }

    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(CLAW_OPEN);
            packet.put("ClawPos", claw.getPosition());
            return false;
        }
    }

    public Action ClawOpen() {
        return new ClawOpen();
    }

    public class ClawToggle implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (claw.getPosition() > 0.2) {
                claw.setPosition(CLAW_CLOSE);
            }
            else {
                claw.setPosition(CLAW_OPEN);
            };

            packet.put("ClawPos", claw.getPosition());
            return false;
        }
    }

    public Action ClawToggle() {
        return new ClawToggle();
    }

}