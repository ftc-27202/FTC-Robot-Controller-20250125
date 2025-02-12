package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public final class Bot_Slides {
    final int SLIDE_GROUND = 0;
    final int SLIDE_CATCH = 500;
    final int SLIDE_CLEAR_ARM = 900;
    final int SLIDE_ASCEND = 1100;
    final int SLIDE_CLEAR_ARM_AUTO_SPECIMEN = 1200;
    final int SLIDE_HIGH = 2650;
    final double SLIDE_STALL_TIME = 2.0;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;

    private float desiredAdjustment = 0;

    public Bot_Slides(HardwareMap hardwareMap) {
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setPower(1.0);
        rightSlide.setPower(1.0);
        leftSlide.setTargetPosition(SLIDE_GROUND);
        rightSlide.setTargetPosition(SLIDE_GROUND);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean SlidesPositionRaisedHigh () {
        if (leftSlide.getCurrentPosition() >= SLIDE_CLEAR_ARM_AUTO_SPECIMEN) return true;
        else return false;
    }

    public class SlidesUpHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide < SLIDE_HIGH) {
                leftSlide.setTargetPosition(SLIDE_HIGH);
                rightSlide.setTargetPosition(SLIDE_HIGH);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesUpHigh() {
        return new SlidesUpHigh();
    }

    public class SlidesDownGround implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(0.80);
                rightSlide.setPower(0.80);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide > (SLIDE_GROUND + 100)) {
                leftSlide.setTargetPosition(SLIDE_GROUND);
                rightSlide.setTargetPosition(SLIDE_GROUND);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesDownGround() {
        return new SlidesDownGround();
    }

    public class SlidesClearArm implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide < (SLIDE_CLEAR_ARM - 100)) {
                leftSlide.setTargetPosition(SLIDE_CLEAR_ARM);
                rightSlide.setTargetPosition(SLIDE_CLEAR_ARM);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesClearArm() {
        return new SlidesClearArm();
    }

    public class SlidesClearArmAutoSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide < (SLIDE_CLEAR_ARM_AUTO_SPECIMEN - 100)) {
                leftSlide.setTargetPosition(SLIDE_CLEAR_ARM_AUTO_SPECIMEN);
                rightSlide.setTargetPosition(SLIDE_CLEAR_ARM_AUTO_SPECIMEN);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesClearArmAutoSpecimen() {
        return new SlidesClearArmAutoSpecimen();
    }

    public class SlidesDownCatch implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(0.80);
                rightSlide.setPower(0.80);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide > (SLIDE_CATCH)) {
                leftSlide.setTargetPosition(SLIDE_CATCH);
                rightSlide.setTargetPosition(SLIDE_CATCH);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesDownCatch() {
        return new SlidesDownCatch();
    }

    public class SlidesUpCatch implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide < (SLIDE_CATCH)) {
                leftSlide.setTargetPosition(SLIDE_CATCH);
                rightSlide.setTargetPosition(SLIDE_CATCH);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesUpCatch() {
        return new SlidesUpCatch();
    }

    public class SlidesUpAscend implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            if (posRightSlide < SLIDE_ASCEND) {
                leftSlide.setTargetPosition(SLIDE_ASCEND);
                rightSlide.setTargetPosition(SLIDE_ASCEND);
                return true;
            } else {
                return false;
            }
        }
    }

    public Action SlidesUpAscend() {
        return new SlidesUpAscend();
    }

    public class ResetSlides implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(0.0);
                rightSlide.setPower(0.0);
                initialized = true;
            }

            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }
    }

    public Action ResetSlides() {
        return new ResetSlides();
    }

    public class MoveSlides implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftSlide.setPower(1.0);
                rightSlide.setPower(1.0);
                initialized = true;
            }

            double posLeftSlide = leftSlide.getCurrentPosition();
            double posRightSlide = rightSlide.getCurrentPosition();
            packet.put("posLeftSlide", posLeftSlide);
            packet.put("posRightSlide", posRightSlide);
            leftSlide.setTargetPosition((int) posLeftSlide + (int) Math.round(desiredAdjustment));
            rightSlide.setTargetPosition((int) posRightSlide + (int) Math.round(desiredAdjustment));
            return false;
        }
    }

    public Action MoveSlides(float adjustment) {
        desiredAdjustment = adjustment;
        return new MoveSlides();
    }
}
