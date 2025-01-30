package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public final class Bot_Gripper {
    // For physical install, 0.5 = Gripper middle position
    final double GRIPPER_IN = 0;
    final double GRIPPER_GRABBING_INWARDS = 0.25;
    final double GRIPPER_GRABBING_OUTWARDS = 1.0;
    final double GRIPPER_OUT = 1.0;

    private Servo gripper;

    public Bot_Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(Servo.class, "gripper");
    }

    public class GripperIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_IN);
            return false;
        }
    }

    public Action GripperIn() {
        return new GripperIn();
    }

    public class GripperGrabOutwards implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_GRABBING_OUTWARDS);
            return false;
        }
    }

    public Action GripperGrabOutwards() {
        return new GripperGrabOutwards();
    }

    public class GripperGrabInwards implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_GRABBING_INWARDS);
            return false;
        }
    }

    public Action GripperGrabInwards() {
        return new GripperGrabInwards();
    }

    public class GripperOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPosition(GRIPPER_OUT);
            return false;
        }
    }

    public Action GripperOut() {
        return new GripperOut();
    }
}