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
@TeleOp(name = "Test Bucket", group = "Test")
public class test_bucket extends LinearOpMode {
    final double BUCKET_CATCH = 0.36;
    final double BUCKET_DUMP = 0.0;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private ServoImplEx bucket;

    @Override
    public void runOpMode() {
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                bucket.setPosition(BUCKET_CATCH);
            } else if (gamepad1.b) {
                bucket.setPosition(BUCKET_DUMP);
            } else if (gamepad1.left_bumper) {
                bucket.setPosition(-gamepad1.left_stick_y);
            }

            telemetry.addData("Status", "Initialized");
            telemetry.addData("gamepad1.a", "= BUCKET_CATCH");
            telemetry.addData("gamepad1.b", "= BUCKET_DUMP");
            telemetry.addData("gamepad1.left_bumper && left_stick_y", "= manual control");
            telemetry.addData("-gamepad1.left_stick_y", -gamepad1.left_stick_y);
            telemetry.addData("bucket.getPosition", bucket.getPosition());
            telemetry.update();
        }
    }
}