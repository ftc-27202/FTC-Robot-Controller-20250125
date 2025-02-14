/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@TeleOp(name = "Test Wrist Rotation", group = "Test")

public class test_wristRotation extends LinearOpMode {
    final double WRIST_ROTATION_VERTICAL_ALIGNMENT = 0.98;
    final double WRIST_ROTATION_HORIZONTAL_ALIGNMENT = 0.60;
    final double WRIST_ROTATION_SPECIMEN = 0.20;
    final double WRIST_ROTATION_AUTO_SAMPLE_3 = 0.85;

//    final double WRIST_ROTATION_VERTICAL_ALIGNMENT = 0.0;
//    final double WRIST_ROTATION_HORIZONTAL_ALIGNMENT = 0.5;
    private double posRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;
    private double posPriorRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private ServoImplEx wristRotation;

    @Override
    public void runOpMode() {
        wristRotation = hardwareMap.get(ServoImplEx.class, "wristRotation");
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                posRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;
                posPriorRotation = posRotation;
                wristRotation.setPosition(WRIST_ROTATION_VERTICAL_ALIGNMENT);
            } else if (gamepad1.b) {
                posRotation = WRIST_ROTATION_HORIZONTAL_ALIGNMENT;
                posPriorRotation = posRotation;
                wristRotation.setPosition(WRIST_ROTATION_HORIZONTAL_ALIGNMENT);
            } else if (gamepad1.x) {
                wristRotation.setPosition(WRIST_ROTATION_SPECIMEN);
            } else if (gamepad1.y) {
                wristRotation.setPosition(WRIST_ROTATION_AUTO_SAMPLE_3);
            } else if (gamepad1.right_trigger > 0) {
                if (posPriorRotation == WRIST_ROTATION_HORIZONTAL_ALIGNMENT) {
                    posRotation -= gamepad1.right_trigger * 0.0017;
                } else {
                    posRotation += gamepad1.right_trigger * 0.0017;
                };
                if (posRotation > WRIST_ROTATION_HORIZONTAL_ALIGNMENT) {
                    posRotation = WRIST_ROTATION_HORIZONTAL_ALIGNMENT;
                };
                if (posRotation < WRIST_ROTATION_VERTICAL_ALIGNMENT){
                    posRotation = WRIST_ROTATION_VERTICAL_ALIGNMENT;
                };

                wristRotation.setPosition(posRotation);
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("gamepad1.a (vertical)", WRIST_ROTATION_VERTICAL_ALIGNMENT);
            telemetry.addData("gamepad1.b (horizontal", WRIST_ROTATION_HORIZONTAL_ALIGNMENT);
            telemetry.addData("gamepad1.x (WRIST_ROTATION_SPECIMEN)", WRIST_ROTATION_SPECIMEN);
            telemetry.addData("gamepad1.y (WRIST_ROTATION_AUTO_SAMPLE_3", WRIST_ROTATION_AUTO_SAMPLE_3);
            telemetry.addData("gamepad1.right_trigger", "= manual control");
            telemetry.addData("wristRotation.getPosition", wristRotation.getPosition());
            telemetry.addData("gamepad1.right_trigger", gamepad1.right_trigger);
            telemetry.addData("posRotation", posRotation);
            telemetry.update();
        }
    }
}