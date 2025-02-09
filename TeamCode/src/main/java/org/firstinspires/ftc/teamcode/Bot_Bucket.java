package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
public final class Bot_Bucket {
    final double BUCKET_CATCH = 0.43;
    final double BUCKET_DUMP = 0.0;

    private ServoImplEx bucket;

    public Bot_Bucket(HardwareMap hardwareMap) {
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
    }

    public class BucketDump implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucket.setPwmEnable();
            bucket.setPosition(BUCKET_DUMP);
            packet.put("BucketPos", bucket.getPosition());
            return false;
        }
    }

    public Action BucketDump() {
        return new BucketDump();
    }

    public class BucketCatch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucket.setPwmEnable();
            bucket.setPosition(BUCKET_CATCH);
            packet.put("BucketPos", bucket.getPosition());
            return false;
        }
    }

    public Action BucketCatch() {
        return new BucketCatch();
    }
    public class BucketOff implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            bucket.setPwmDisable();
            return false;
        }
    }

    public Action BucketOff() {
        return new BucketOff();
    }
}