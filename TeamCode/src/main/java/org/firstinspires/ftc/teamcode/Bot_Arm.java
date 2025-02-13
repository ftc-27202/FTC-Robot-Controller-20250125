package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Bot_Arm {
    public double ARM_FUDGE = 1.75;

    public double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1 / 360.0; // Ticks per degree, not per rotation
    public double ARM_COLLAPSED_INTO_ROBOT = 0;
    public double ARM_DOWN_FOR_ASCENDING = (50) * ARM_TICKS_PER_DEGREE;
    public double ARM_SPECIMEN_BEFORE_SCORE = (68) * ARM_TICKS_PER_DEGREE;
    public double ARM_DROP_SAMPLE_TO_ZONE = (73) * ARM_TICKS_PER_DEGREE;
    public double ARM_DEPOSIT = (91) * ARM_TICKS_PER_DEGREE;
    public double ARM_STRAIGHT_UP = (90) * ARM_TICKS_PER_DEGREE;
    public double ARM_CLEAR_BUCKET = (100)* ARM_TICKS_PER_DEGREE;
    public double ARM_PREPARE_TO_ASCEND = (100)* ARM_TICKS_PER_DEGREE;
    public double ARM_SPECIMEN_AFTER_SCORE = (115) * ARM_TICKS_PER_DEGREE;
    public double ARM_PREPARE_TO_COLLECT_SPECIMEN_AUTO = (170) * ARM_TICKS_PER_DEGREE;
    public double ARM_PREPARE_TO_COLLECT = (174) * ARM_TICKS_PER_DEGREE; // almost parallel to the ground, just above specimen's height
    public double ARM_COLLECTED = ARM_PREPARE_TO_COLLECT;
    public double ARM_COLLECT_SPECIMEN = (177.5) * ARM_TICKS_PER_DEGREE;
    public double ARM_SHOVE = (184)* ARM_TICKS_PER_DEGREE;
    public double ARM_COLLECT_SAMPLE = (190) * ARM_TICKS_PER_DEGREE;

    private DcMotorEx armMotor;
    private float desiredAdjustment = 0;

    public Bot_Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition((int) ARM_COLLAPSED_INTO_ROBOT);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(2100);
    }
    public boolean ArmUpForDeposit () {
        if ((armMotor.getCurrentPosition() <= ARM_DEPOSIT)&&(armMotor.getCurrentPosition() >= ARM_SPECIMEN_BEFORE_SCORE)) return true;
        else return false;
    }
    public boolean ArmWithinBucketClearance() {
        double pos = armMotor.getCurrentPosition();

        if (pos > (ARM_DEPOSIT - 3 * ARM_TICKS_PER_DEGREE) && pos < (ARM_DEPOSIT + 3 * ARM_TICKS_PER_DEGREE)) {
            return false;
        }
        else return true;
    }

    public class ArmFudgeUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                ARM_FUDGE = ARM_FUDGE + .25;
            }

            return false;
        }
    }

    public Action ArmFudgeUp() {
        return new ArmFudgeUp();
    }
    public class ArmFudgeDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                initialized = true;
                ARM_FUDGE = ARM_FUDGE - .25;
            }

            return false;
        }
    }
    public Action ArmFudgeDown() {
        return new ArmFudgeDown();
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
            if (pos < (ARM_COLLECT_SAMPLE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) )- 5) {  // 5 is the buffer to avoid delay
                armMotor.setTargetPosition((int)((ARM_COLLECT_SAMPLE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos > ARM_COLLECTED+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_COLLECTED+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollected() {
        return new ArmCollected();
    }

    public class ArmShove implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.5);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_SHOVE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE)  - 3) {  // 3 is the buffer to avoid delay
                armMotor.setTargetPosition((int)((ARM_SHOVE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmShove() {
        return new ArmShove();
    }

    public class ArmCollectSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.20);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_COLLECT_SPECIMEN+(ARM_FUDGE*ARM_TICKS_PER_DEGREE)  - 5) {  // 5 is the buffer to avoid delay
                armMotor.setTargetPosition((int)((ARM_COLLECT_SPECIMEN+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos > ARM_DEPOSIT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_DEPOSIT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDeposit() {
        return new ArmDeposit();
    }

    public class ArmPrepareToAscend implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_PREPARE_TO_ASCEND+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_PREPARE_TO_ASCEND+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmPrepareToAscend() {
        return new ArmPrepareToAscend();
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
            if (pos < ARM_CLEAR_BUCKET+(ARM_FUDGE*ARM_TICKS_PER_DEGREE)  - 2) {
                armMotor.setTargetPosition((int)((ARM_CLEAR_BUCKET+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos > ARM_COLLAPSED_INTO_ROBOT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_COLLAPSED_INTO_ROBOT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmCollapsedIntoRobot() {
        return new ArmCollapsedIntoRobot();
    }

    public class ArmDownForAscending implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos > ARM_DOWN_FOR_ASCENDING+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_DOWN_FOR_ASCENDING+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDownForAscending() {
        return new ArmDownForAscending();
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
            if (pos < ARM_PREPARE_TO_COLLECT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_PREPARE_TO_COLLECT+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmPrepareToCollect() {
        return new ArmPrepareToCollect();
    }

    public class ArmPrepareToCollectSpecimenAuto implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            packet.put("armMotorPos", pos / ARM_TICKS_PER_DEGREE);
            if (pos < ARM_PREPARE_TO_COLLECT_SPECIMEN_AUTO+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_PREPARE_TO_COLLECT_SPECIMEN_AUTO+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmPrepareToCollectSpecimenAuto() {
        return new ArmPrepareToCollectSpecimenAuto();
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
            if (pos < ARM_STRAIGHT_UP+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_STRAIGHT_UP+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos < ARM_SPECIMEN_BEFORE_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_SPECIMEN_BEFORE_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos > ARM_SPECIMEN_BEFORE_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_SPECIMEN_BEFORE_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
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
            if (pos < ARM_SPECIMEN_AFTER_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_SPECIMEN_AFTER_SCORE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmSpecimenAfterScore() {
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
            if (pos > ARM_DROP_SAMPLE_TO_ZONE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE) ) {
                armMotor.setTargetPosition((int)((ARM_DROP_SAMPLE_TO_ZONE+(ARM_FUDGE*ARM_TICKS_PER_DEGREE))));
                return true;
            } else {
                return false;
            }
        }
    }

    public Action ArmDropSampleToZone() {
        return new ArmDropSampleToZone();
    }

    public class ArmReset implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(0.0);
                initialized = true;
            }

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action ArmReset() {
        return new ArmReset();
    }

    public class MoveArm implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                armMotor.setPower(1.0);
                initialized = true;
            }

            double posArm = armMotor.getCurrentPosition();
            packet.put("armMotorPos", posArm / ARM_TICKS_PER_DEGREE);
            armMotor.setTargetPosition((int) posArm + (int) Math.round(desiredAdjustment * ARM_TICKS_PER_DEGREE));
            return false;
        }
    }

    public Action MoveArm(float position) {
        desiredAdjustment = position;
        return new MoveArm();
    }

}

