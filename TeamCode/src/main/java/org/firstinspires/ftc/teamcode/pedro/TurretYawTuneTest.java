package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Turret Yaw Tune TEST", group = "Test")
public class TurretYawTuneTest extends LinearOpMode {

    CRServo tl, tr;
    Limelight3A limelight;

    // 👇 YOU WILL TUNE THIS
    double yawOffset = 0.0;

    static final int TAG_ID = 24; // change to 20 on blue if needed
    static final double KP = 0.015;
    static final double DEADZONE = 1.5;
    static final double MIN = 0.05;
    static final double MAX = 0.3;

    @Override
    public void runOpMode() {

        tl = hardwareMap.get(CRServo.class, "tl");
        tr = hardwareMap.get(CRServo.class, "tr");

        for (Limelight3A ll : hardwareMap.getAll(Limelight3A.class)) {
            limelight = ll;
            break;
        }

        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            // 🔧 ADJUST OFFSET LIVE
            if (gamepad1.dpad_left)  yawOffset -= 0.05;
            if (gamepad1.dpad_right) yawOffset += 0.05;

            LLResult r = limelight.getLatestResult();

            if (r == null || !r.isValid()) {
                tl.setPower(0);
                tr.setPower(0);
                telemetry.addLine("NO TAG");
                telemetry.update();
                continue;
            }

            boolean found = false;

            for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
                if (t.getFiducialId() == TAG_ID) {
                    found = true;

                    double tx = t.getTargetXDegrees() + yawOffset;

                    double power = 0;
                    if (Math.abs(tx) > DEADZONE) {
                        power = Math.signum(tx) *
                                Math.min(MAX, Math.max(MIN, Math.abs(tx * KP)));
                    }

                    tl.setPower(power);
                    tr.setPower(power);

                    telemetry.addData("RAW tx", t.getTargetXDegrees());
                    telemetry.addData("OFFSET", yawOffset);
                    telemetry.addData("USED tx", tx);
                    telemetry.addData("POWER", power);
                    break;
                }
            }

            if (!found) {
                tl.setPower(0);
                tr.setPower(0);
                telemetry.addLine("TAG NOT FOUND");
            }

            telemetry.update();
        }
    }
}
