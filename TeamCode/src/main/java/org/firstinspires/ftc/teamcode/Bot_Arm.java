package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Bot_Arm {
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation
    final double ARM_COLLAPSED_INTO_ROBOT = 0;
    final double ARM_DROP_SAMPLE_TO_ZONE = 70 * ARM_TICKS_PER_DEGREE;
    final double ARM_DEPOSIT = 88 * ARM_TICKS_PER_DEGREE;
    final double ARM_STRAIGHT_UP = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BUCKET = 100 * ARM_TICKS_PER_DEGREE;
    final double ARM_SPECIMEN_BEFORE_SCORE = 85 * ARM_TICKS_PER_DEGREE;
    final double ARM_SPECIMEN_AFTER_SCORE = 115 * ARM_TICKS_PER_DEGREE;
    final double ARM_PREPARE_TO_COLLECT = 176 * ARM_TICKS_PER_DEGREE; // parallel to the ground
    final double ARM_COLLECTED = 176 * ARM_TICKS_PER_DEGREE; // parallel to the ground
    final double ARM_COLLECT_SPECIMEN = 185 * ARM_TICKS_PER_DEGREE;
    final double ARM_COLLECT_SAMPLE = 188 * ARM_TICKS_PER_DEGREE;

    private DcMotorEx armMotor;

    public Bot_Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
    }

    public class ArmCollectSample implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.50);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_COLLECT_SAMPLE - 5) {  // 5 is the buffer to avoid delay
                armMotor.setTargetPosition((int) ARM_COLLECT_SAMPLE);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollectSample() {
        return new ArmCollectSample();
    }

    public class ArmCollected implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.50);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_COLLECTED) {
                armMotor.setTargetPosition((int) ARM_COLLECTED);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollected() {
        return new ArmCollected();
    }

    public class ArmCollectSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.50);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_COLLECT_SPECIMEN - 5) {  // 5 is the buffer to avoid delay
                armMotor.setTargetPosition((int) ARM_COLLECT_SPECIMEN);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollectSpecimen() {
        return new ArmCollectSpecimen();
    }

    public class ArmDeposit implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_DEPOSIT) {
                armMotor.setTargetPosition((int) ARM_DEPOSIT);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDeposit() {
        return new ArmDeposit();
    }

    public class ArmClearBucket implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_CLEAR_BUCKET - 2) {
                armMotor.setTargetPosition((int) ARM_CLEAR_BUCKET);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmClearBucket() {
        return new ArmClearBucket();
    }

    public class ArmCollapsedIntoRobot implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_COLLAPSED_INTO_ROBOT) {
                armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollapsedIntoRobot() {
        return new ArmCollapsedIntoRobot();
    }

    public class ArmPrepareToCollect implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_PREPARE_TO_COLLECT) {
                armMotor.setTargetPosition((int) ARM_PREPARE_TO_COLLECT);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmPrepareToCollect() {
        return new ArmPrepareToCollect();
    }

    public class ArmStraightUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_STRAIGHT_UP) {
                armMotor.setTargetPosition((int) ARM_STRAIGHT_UP);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmStraightUp() {
        return new ArmStraightUp();
    }

    public class ArmDownSpecimenBeforeScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_SPECIMEN_BEFORE_SCORE) {
                armMotor.setTargetPosition((int) ARM_SPECIMEN_BEFORE_SCORE);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDownSpecimenBeforeScore() {
        return new ArmDownSpecimenBeforeScore();
    }

    public class ArmUpSpecimenBeforeScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_SPECIMEN_BEFORE_SCORE) {
                armMotor.setTargetPosition((int) ARM_SPECIMEN_BEFORE_SCORE);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmUpSpecimenBeforeScore() {
        return new ArmUpSpecimenBeforeScore();
    }

    public class ArmSpecimenAfterScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_SPECIMEN_AFTER_SCORE) {
                armMotor.setTargetPosition((int) ARM_SPECIMEN_AFTER_SCORE);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmSpecimenScore() {
        return new ArmSpecimenAfterScore();
    }


    public class ArmDropSampleToZone implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_DROP_SAMPLE_TO_ZONE) {
                armMotor.setTargetPosition((int) ARM_DROP_SAMPLE_TO_ZONE);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDropSampleToZone() {
        return new ArmDropSampleToZone();
    }

}

