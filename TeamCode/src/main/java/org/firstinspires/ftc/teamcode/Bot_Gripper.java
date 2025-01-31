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
    final double GRIPPER_GRABBING_INWARDS = 0.25;
    final double GRIPPER_HALFWAY_OPEN = 0.60;
    final double GRIPPER_OUT = 0.85;

    private ServoImplEx gripper;

    public Bot_Gripper(HardwareMap hardwareMap) {
        gripper = hardwareMap.get(ServoImplEx.class, "gripper");
    }

    public class GripperGrabInwards implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            gripper.setPwmEnable();
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
            gripper.setPwmEnable();
            gripper.setPosition(GRIPPER_OUT);
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
            gripper.setPwmDisable();
            return false;
        }
    }

    public Action GripperOpenHalfway() {
        return new GripperOpenHalfway();
    }

}