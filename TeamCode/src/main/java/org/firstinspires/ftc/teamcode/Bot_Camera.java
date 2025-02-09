package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Config
public final class Bot_Camera {
    final int LIMELIGHT_PIPELINE_RED_SPECIMEN_INDEX = 2;
    final int LIMELIGHT_PIPELINE_BLUE_SPECIMEN_INDEX = 3;
    final int LIMELIGHT_PIPELINE_YELLOW_HORIZONTAL_INDEX = 4;
    final int LIMELIGHT_PIPELINE_RED_HORIZONTAL_INDEX = 5;
    final int LIMELIGHT_PIPELINE_BLUE_HORIZONTAL_INDEX = 6;
    final int LIMELIGHT_PIPELINE_YELLOW_VERTICAL_INDEX = 7;
    final int LIMELIGHT_PIPELINE_RED_VERTICAL_INDEX = 8;
    final int LIMELIGHT_PIPELINE_BLUE_VERTICAL_INDEX = 9;
    public LLResult LimelightResult;
    public LLStatus LimelightStatus;
    public static class Params {
        public double ANGLE_TO_DISTANCE_X_FACTOR = 0.17;  // conversion for Limelight degrees to inches (very crude)
        public double ANGLE_TO_DISTANCE_Y_FACTOR = 0.07;  // conversion for Limelight degrees to inches (very crude)
    }

    public static Bot_Camera.Params PARAMS = new Bot_Camera.Params();

    double crosshair_x;
    double crosshair_y;
    double crosshair_angle;

    public Limelight3A limelight3A;  // physically installed as upside down

    public Bot_Camera(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.start();
    }

    public void StopCamera() {
        limelight3A.stop();
    }

    public Vector2d ObtainCrosshair(String color, String orientation) {
        Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Vector2d crosshair = new Vector2d(0, 0);

        limelight3A.start();

        if (color.equals("YELLOW") && orientation.equals("VERTICAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_YELLOW_VERTICAL_INDEX);
        }
        else if (color.equals("YELLOW") && orientation.equals("HORIZONTAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_YELLOW_HORIZONTAL_INDEX);
        }
        else if (color.equals("RED") && orientation.equals("VERTICAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_RED_VERTICAL_INDEX);
        }
        else if (color.equals("RED") && orientation.equals("HORIZONTAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_RED_HORIZONTAL_INDEX);
        }
        else if (color.equals("BLUE") && orientation.equals("VERTICAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_BLUE_VERTICAL_INDEX);
        }
        else if (color.equals("BLUE") && orientation.equals("HORIZONTAL")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_BLUE_HORIZONTAL_INDEX);
        }
        else if (color.equals("RED") && orientation.equals("VERTICAL_SPECIMEN")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_RED_SPECIMEN_INDEX);
        }
        else if (color.equals("BLUE") && orientation.equals("VERTICAL_SPECIMEN")) {
            limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_BLUE_SPECIMEN_INDEX);
        }
        sleep(100);
        LLStatus status = limelight3A.getStatus();
        LimelightStatus = status;
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult limelight_result = limelight3A.getLatestResult();
        LimelightResult = limelight_result;
        crosshair_x = 0;
        crosshair_y = 0;
        if (limelight_result != null) {
            // Access general information
            Pose3D botpose = limelight_result.getBotpose();
            double captureLatency = limelight_result.getCaptureLatency();
            double targetingLatency = limelight_result.getTargetingLatency();
            double parseLatency = limelight_result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(limelight_result.getPythonOutput()));

            if (limelight_result.isValid()) {
                telemetry.addData("tx", limelight_result.getTx());
                telemetry.addData("txnc", limelight_result.getTxNC());
                telemetry.addData("ty", limelight_result.getTy());
                telemetry.addData("tync", limelight_result.getTyNC());
                telemetry.addData("Botpose", botpose.toString());

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = limelight_result.getColorResults();
                LLResultTypes.ColorResult cr = colorResults.get(0);

                telemetry.addData("Color (degrees)", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                telemetry.addData("Pixel (pixels])", "X: %.2f, Y: %.2f", cr.getTargetXPixels(), cr.getTargetYPixels());

//                crosshair_x = 8 * Math.tan(cr.getTargetXDegrees());
                crosshair_x = (cr.getTargetXDegrees() * -1) * PARAMS.ANGLE_TO_DISTANCE_X_FACTOR ;  // multiply by negative 1 since the camera is upside down
                crosshair_y = (cr.getTargetYDegrees() * -1) * PARAMS.ANGLE_TO_DISTANCE_Y_FACTOR;  // less 1 inch, since limelight's crosshair is set to Bottom (unrotated)
                crosshair_angle = 0;

                telemetry.addData("ANGLE_TO_DISTANCE_X_FACTOR", PARAMS.ANGLE_TO_DISTANCE_X_FACTOR);
                telemetry.addData("ANGLE_TO_DISTANCE_Y_FACTOR", PARAMS.ANGLE_TO_DISTANCE_Y_FACTOR);
                telemetry.addData("Crosshair (fudged inches)", "X: %.2f, Y: %.2f", crosshair_x, crosshair_y);
            }
            else {
            telemetry.addData("Limelight", "No data available");
        }
            telemetry.update();
        }

        crosshair = new Vector2d(crosshair_x, crosshair_y);
        return crosshair;
    }
}