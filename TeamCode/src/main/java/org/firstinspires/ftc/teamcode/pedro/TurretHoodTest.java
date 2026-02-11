package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret + Hood Test", group = "Test")
public class TurretHoodTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo ha;
    private CRServo tl, tr;
    private Limelight3A limelight;

    // ---- TURRET FILTERING / ANTI-VIBE ----
    double filteredTx = 0;

    private static final double TX_ALPHA = 0.2;        // smoothing (lower = smoother)
    private static final double TRACK_KP = 0.012;      // normal tracking
    private static final double TRACK_KP_SHOOTING = 0.006; // when shooter on

    private static final double TRACK_MAX_POWER = 0.25;
    private static final double TRACK_MAX_POWER_SHOOTING = 0.12;

    // ignore tiny vibration noise (NOT a deadzone)
    private static final double NOISE_IGNORE_DEG = 0.4;

    // ---- HOOD ADJUSTMENT BASED ON DISTANCE ----
    private static final double HOOD_CLOSE = 0.4;  // close range hood position
    private static final double HOOD_FAR = 0.7;    // far range hood position

    // Distance thresholds (based on ty - vertical angle from limelight)
    private static final double TY_CLOSE_THRESHOLD = 10.0;  // if ty > 10, target is close
    private static final double TY_FAR_THRESHOLD = -5.0;    // if ty < -5, target is far

    // AprilTag targeting
    private int targetTagId = 20; // Blue by default, change to 24 for Red

    // Test modes
    private boolean autoHoodEnabled = true;
    private boolean shooterSimulationActive = false;

    @Override
    public void runOpMode() {

        // ---- HARDWARE INITIALIZATION ----
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        ha = hardwareMap.get(Servo.class, "ha");
        tl = hardwareMap.get(CRServo.class, "tl");
        tr = hardwareMap.get(CRServo.class, "tr");

        for (Limelight3A ll : hardwareMap.getAll(Limelight3A.class)) {
            limelight = ll;
            break;
        }
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
        }

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ha.setPosition(HOOD_CLOSE);

        telemetry.addLine("=== TURRET + HOOD TEST ===");
        telemetry.addLine("Gamepad 1:");
        telemetry.addLine("  Left stick = Drive");
        telemetry.addLine("  Right stick X = Rotate");
        telemetry.addLine("  DPAD Left/Right = Manual turret");
        telemetry.addLine("  A = Toggle Auto Hood");
        telemetry.addLine("  B = Toggle Shooter Simulation");
        telemetry.addLine("  X/Y = Manual hood adjust");
        telemetry.addLine("");
        telemetry.addLine("Default: Blue Alliance (Tag 20)");
        telemetry.addLine("Press DPAD UP for Red Alliance (Tag 24)");
        telemetry.update();

        boolean lastDpadUp = false;

        while (!isStarted() && !isStopRequested()) {
            boolean currentDpadUp = gamepad1.dpad_up;

            if (currentDpadUp && !lastDpadUp) {
                targetTagId = (targetTagId == 20) ? 24 : 20;
            }

            lastDpadUp = currentDpadUp;

            telemetry.addLine("=== ALLIANCE SELECTION ===");
            telemetry.addData("Current", targetTagId == 20 ? "BLUE (Tag 20)" : "RED (Tag 24)");
            telemetry.addLine("");
            telemetry.addLine("Press START when ready");
            telemetry.update();
        }

        boolean lastA = false, lastB = false, lastX = false, lastY = false;
        double manualHoodPos = HOOD_CLOSE;

        waitForStart();

        while (opModeIsActive()) {

            // ---- DRIVE ----
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x * 1.5;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ---- TOGGLE AUTO HOOD ----
            if (gamepad1.a && !lastA) {
                autoHoodEnabled = !autoHoodEnabled;
            }
            lastA = gamepad1.a;

            // ---- TOGGLE SHOOTER SIMULATION ----
            if (gamepad1.b && !lastB) {
                shooterSimulationActive = !shooterSimulationActive;
            }
            lastB = gamepad1.b;

            // ---- MANUAL HOOD ADJUSTMENT (when auto is off) ----
            if (!autoHoodEnabled) {
                if (gamepad1.x && !lastX) manualHoodPos += 0.05;
                if (gamepad1.y && !lastY) manualHoodPos -= 0.05;
                manualHoodPos = Math.max(0, Math.min(1, manualHoodPos));
                ha.setPosition(manualHoodPos);
            }
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // ---- TURRET + HOOD TRACKING ----
            boolean manualTurret = gamepad1.dpad_left || gamepad1.dpad_right;

            if (manualTurret) {
                double p = 0.5;
                if (gamepad1.dpad_left) {
                    tl.setPower(-p);
                    tr.setPower(-p);
                } else if (gamepad1.dpad_right) {
                    tl.setPower(p);
                    tr.setPower(p);
                } else {
                    tl.setPower(0);
                    tr.setPower(0);
                }
            } else {
                autoTrackAprilTag();
            }

            // ---- TELEMETRY ----
            telemetry.addLine("=== TURRET + HOOD TEST ===");
            telemetry.addData("Target Tag", targetTagId == 20 ? "BLUE (20)" : "RED (24)");
            telemetry.addData("Auto Hood", autoHoodEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Shooter Sim", shooterSimulationActive ? "ON" : "OFF");
            telemetry.addData("Hood Position", "%.2f", autoHoodEnabled ? ha.getPosition() : manualHoodPos);
            telemetry.addData("Filtered TX", "%.2f", filteredTx);

            if (limelight != null) {
                LLResult r = limelight.getLatestResult();
                if (r != null && r.isValid()) {
                    for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
                        if (t.getFiducialId() == targetTagId) {
                            telemetry.addData("Tag TX", "%.2f", t.getTargetXDegrees());
                            telemetry.addData("Tag TY", "%.2f", t.getTargetYDegrees());
                            telemetry.addData("Distance", estimateDistance(t.getTargetYDegrees()));
                        }
                    }
                } else {
                    telemetry.addLine("NO TAG DETECTED");
                }
            }

            telemetry.update();
        }
    }

    private void autoTrackAprilTag() {

        if (limelight == null) {
            tl.setPower(0);
            tr.setPower(0);
            return;
        }

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) {
            tl.setPower(0);
            tr.setPower(0);
            return;
        }

        for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {

            if (t.getFiducialId() != targetTagId) continue;

            // ---- RAW TX ----
            double rawTx = t.getTargetXDegrees();

            // ---- LOW PASS FILTER (ANTI VIBRATION) ----
            filteredTx = filteredTx * (1 - TX_ALPHA) + rawTx * TX_ALPHA;
            double tx = filteredTx;

            // ---- IGNORE NOISE ----
            if (Math.abs(tx) < NOISE_IGNORE_DEG) {
                tl.setPower(0);
                tr.setPower(0);
            } else {
                // ---- SHOOTER-AWARE GAIN ----
                double kp = shooterSimulationActive ? TRACK_KP_SHOOTING : TRACK_KP;

                double power = tx * kp;

                // ---- HARD CLAMP ----
                double maxP = shooterSimulationActive ? TRACK_MAX_POWER_SHOOTING : TRACK_MAX_POWER;
                power = Math.max(-maxP, Math.min(maxP, power));

                tl.setPower(power);
                tr.setPower(power);
            }

            // ---- AUTO HOOD ADJUSTMENT BASED ON DISTANCE ----
            if (autoHoodEnabled) {
                double ty = t.getTargetYDegrees();

                // Smooth interpolation: farther away (lower ty) = higher hood
                // Clamp ty between thresholds, then map to hood range
                double clampedTy = Math.max(TY_FAR_THRESHOLD, Math.min(TY_CLOSE_THRESHOLD, ty));

                // Calculate ratio: 0.0 at close, 1.0 at far
                double ratio = (TY_CLOSE_THRESHOLD - clampedTy) / (TY_CLOSE_THRESHOLD - TY_FAR_THRESHOLD);

                // Map ratio to hood position: 0.4 close → 0.7 far
                double hoodPos = HOOD_CLOSE + (HOOD_FAR - HOOD_CLOSE) * ratio;

                ha.setPosition(hoodPos);
            }

            return;
        }

        // no tag
        tl.setPower(0);
        tr.setPower(0);
    }

    private String estimateDistance(double ty) {
        // Calculate what the hood position would be
        double clampedTy = Math.max(TY_FAR_THRESHOLD, Math.min(TY_CLOSE_THRESHOLD, ty));
        double ratio = (TY_CLOSE_THRESHOLD - clampedTy) / (TY_CLOSE_THRESHOLD - TY_FAR_THRESHOLD);
        double hoodPos = HOOD_CLOSE + (HOOD_FAR - HOOD_CLOSE) * ratio;

        return String.format("Hood: %.2f", hoodPos);
    }
}