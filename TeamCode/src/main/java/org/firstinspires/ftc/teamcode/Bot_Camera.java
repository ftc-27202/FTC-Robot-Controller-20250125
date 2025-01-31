package org.firstinspires.ftc.teamcode;

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
    final int LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX = 7;
    final int LIMELIGHT_PIPELINE_AUTO_RED_INDEX = 8;
    final int LIMELIGHT_PIPELINE_AUTO_BLUE_INDEX = 9;

    final double ANGLE_TO_DISTANCE_FACTOR = 0.17;  // conversion for Limelight degrees to inches (very crude)

    double crosshair_x;
    double crosshair_y;
    double crosshair_angle;

    private Limelight3A limelight3A;

    public Bot_Camera(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public Vector2d ObtainCrosshair(String color) {
        Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        Vector2d crosshair = new Vector2d(0, 0);

        limelight3A.start();

        switch (color) {
            case "YELLOW":
                limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_YELLOW_INDEX);
                break;
            case "RED":
                limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_RED_INDEX);
                break;
            case "BLUE":
                limelight3A.pipelineSwitch(LIMELIGHT_PIPELINE_AUTO_BLUE_INDEX);
                break;
        }

        LLStatus status = limelight3A.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult limelight_result = limelight3A.getLatestResult();

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

//                    crosshair_x = 8 * Math.tan(cr.getTargetXDegrees());
                crosshair_x = cr.getTargetXDegrees() * ANGLE_TO_DISTANCE_FACTOR;
                crosshair_y = cr.getTargetYDegrees() * ANGLE_TO_DISTANCE_FACTOR - 1;  // less 1 inch, since limelight's crosshair is set to Top (unrotated)
                crosshair_angle = 0;

                telemetry.addData("ANGLE_TO_DISTANCE_FACTOR", ANGLE_TO_DISTANCE_FACTOR);
                telemetry.addData("Crosshair (fudged inches)", "X: %.2f, Y: %.2f", crosshair_x, crosshair_y);
            }
            else {
            telemetry.addData("Limelight", "No data available");
        }
            telemetry.update();
            limelight3A.stop();
        }

        crosshair = new Vector2d(crosshair_x, crosshair_y);
        return crosshair;
    }
}