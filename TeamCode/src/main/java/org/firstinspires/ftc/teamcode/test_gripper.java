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
@TeleOp(name = "Test Gripper", group = "Test")

public class test_gripper extends LinearOpMode {
    final double GRIPPER_IN = 0.35;
    final double GRIPPER_HALFWAY_OPEN = 0.50;
    final double GRIPPER_OUT = 0.65;
//    final double GRIPPER_GRABBING_INWARDS = 0.35;
//    final double GRIPPER_HALFWAY_OPEN = 0.50;
//    final double GRIPPER_OUT = 0.65;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private ServoImplEx gripper;

    @Override
    public void runOpMode() {
        gripper = hardwareMap.get(ServoImplEx.class, "gripper");
        TelemetryPacket packet = new TelemetryPacket();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                gripper.setPosition(GRIPPER_IN);
            }
            else if (gamepad1.b) {
                gripper.setPosition(GRIPPER_HALFWAY_OPEN);
            }
            else if (gamepad1.y) {
                gripper.setPosition(GRIPPER_OUT);
            }
            else if (gamepad1.left_bumper) {
                gripper.setPosition(-gamepad1.left_stick_y);
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("gamepad1.a", "= GRIPPER_IN");
            telemetry.addData("gamepad1.b", "= GRIPPER_HALFWAY_OPEN");
            telemetry.addData("gamepad1.y", "= GRIPPER_OUT");
            telemetry.addData("gamepad1.left_bumper && left_stick_y", "= manual control");
            telemetry.addData("-gamepad1.left_stick_y", -gamepad1.left_stick_y);
            telemetry.addData("gripper.getPosition", gripper.getPosition());
            telemetry.update();
        }
    }
}