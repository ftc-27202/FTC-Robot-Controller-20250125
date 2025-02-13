package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Gripper {
    // For physical install, 0.5 = Gripper middle position
    final double GRIPPER_IN = 0.3;
    final double GRIPPER_HALFWAY_OPEN = 0.50;
    final double GRIPPER_OUT = 0.65;

    private ServoImplEx gripper;

    public Bot_Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(ServoImplEx.class, "gripper");
    }

    public class GripperIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_IN);
            packet.put("GripperPos", gripper.getPosition());
            return false;
        }
    }

    public Action GripperIn() {
        return new GripperIn();
    }

    public class GripperOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_OUT);
            packet.put("GripperPos", gripper.getPosition());
            return false;
        }
    }

    public Action GripperOut() {
        return new GripperOut();
    }

    public class GripperOpenHalfway implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_HALFWAY_OPEN);
            packet.put("GripperPos", gripper.getPosition());
            return false;
        }
    }

    public Action GripperOpenHalfway() {
        return new GripperOpenHalfway();
    }
}