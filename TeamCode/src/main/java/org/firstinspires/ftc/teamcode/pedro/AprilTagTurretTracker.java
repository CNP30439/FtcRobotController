package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.List;

@TeleOp(name = "AprilTag Turret Tracker", group = "Turret")
public class AprilTagTurretTracker extends OpMode {

    private CRServo leftServo;
    private CRServo rightServo;
    private Limelight3A limelight;

    private static final int TARGET_TAG_ID = 20;
    private static final double DEADZONE = 1.5; // Slightly relaxed for faster tracking
    private static final double MAX_SPEED = 0.7; // Increased for fast response
    private static final double MIN_SPEED = 0.08; // Higher minimum for quick starts

    // Aggressive proportional control for fast tracking
    private static final double KP = 0.035; // Increased gain for faster response

    private boolean trackingEnabled = false;
    private boolean lastAPressed = false;

    // Track consecutive frames on target for stability
    private int framesOnTarget = 0;
    private static final int FRAMES_TO_LOCK = 2; // Reduced for faster locking

    @Override
    public void init() {
        // Initialize CR servos
        leftServo = hardwareMap.get(CRServo.class, "tl");
        rightServo = hardwareMap.get(CRServo.class, "tr");

        stopTurret();

        // Initialize Limelight
        for (Limelight3A ll : hardwareMap.getAll(Limelight3A.class)) {
            limelight = ll;
            break;
        }

        if (limelight == null) {
            telemetry.addData("ERROR", "Limelight not found!");
            telemetry.update();
            return;
        }

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();

        trackingEnabled = true;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Tag", TARGET_TAG_ID);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Edge detection for A button toggle
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAPressed) {
            trackingEnabled = !trackingEnabled;
            if (!trackingEnabled) {
                stopTurret();
                framesOnTarget = 0;
            }
        }
        lastAPressed = aPressed;

        // Manual control with left stick - OVERRIDES tracking
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            trackingEnabled = false;
            framesOnTarget = 0;
            manualControl();
        } else if (!trackingEnabled) {
            stopTurret();
        }

        // Auto-tracking mode
        if (trackingEnabled) {
            autoTrack();
        }

        updateTelemetry();
    }

    private void autoTrack() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            // Find target tag
            LLResultTypes.FiducialResult targetTag = null;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                    targetTag = fiducial;
                    break;
                }
            }

            if (targetTag != null) {
                // TX is the horizontal angle offset from center
                // Positive TX = target is to the RIGHT of center
                // Negative TX = target is to the LEFT of center
                double tx = targetTag.getTargetXDegrees();

                telemetry.addData("TX Raw", "%.2f", tx);
                telemetry.addData("Tag Found", "YES");

                // Check if within deadzone
                if (Math.abs(tx) <= DEADZONE) {
                    framesOnTarget++;
                    if (framesOnTarget >= FRAMES_TO_LOCK) {
                        stopTurret();
                        telemetry.addData("Status", "✓ LOCKED ON TARGET");
                        telemetry.addData("Error", "%.2f° (CENTERED)", tx);
                    } else {
                        // Still settling, keep light correction
                        double speed = tx * KP;
                        speed = clampSpeed(speed);
                        setTurretSpeed(speed);
                        telemetry.addData("Status", "Settling... %d/%d", framesOnTarget, FRAMES_TO_LOCK);
                    }
                } else {
                    // Outside deadzone - need to move
                    framesOnTarget = 0;

                    // Calculate speed based on error
                    // Positive TX (target right) -> positive speed (rotate right)
                    // Negative TX (target left) -> negative speed (rotate left)
                    double speed = tx * KP;

                    // Add minimum speed to overcome friction
                    speed = clampSpeed(speed);

                    setTurretSpeed(speed);

                    telemetry.addData("Status", "TRACKING");
                    telemetry.addData("Error", "%.2f° %s", Math.abs(tx), tx > 0 ? "RIGHT" : "LEFT");
                    telemetry.addData("Speed", "%.3f %s", Math.abs(speed), speed > 0 ? "→" : "←");
                }
            } else {
                stopTurret();
                framesOnTarget = 0;
                telemetry.addData("Status", "Tag 20 not visible");
                telemetry.addData("Tags Seen", fiducials.size());
            }
        } else {
            stopTurret();
            framesOnTarget = 0;
            telemetry.addData("Status", "No AprilTags detected");
        }
    }

    private double clampSpeed(double speed) {
        // Apply minimum speed threshold
        if (Math.abs(speed) > 0.001 && Math.abs(speed) < MIN_SPEED) {
            speed = Math.copySign(MIN_SPEED, speed);
        }

        // Clamp to max speed
        speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speed));

        return speed;
    }

    private void setTurretSpeed(double speed) {
        // Both servos rotate the same direction
        // Positive speed = rotate one way
        // Negative speed = rotate opposite way
        leftServo.setPower(speed);
        rightServo.setPower(speed);
    }

    private void manualControl() {
        double speed = -gamepad1.left_stick_x * 0.5;
        setTurretSpeed(speed);
        telemetry.addData("Mode", "MANUAL OVERRIDE");
        telemetry.addData("Speed", "%.2f", speed);
    }

    private void stopTurret() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addLine("─────────────────────");
        telemetry.addData("Mode", trackingEnabled ? "AUTO-TRACK" : "MANUAL");
        telemetry.addLine();
        telemetry.addData("Controls", "");
        telemetry.addData("  A Button", "Toggle tracking");
        telemetry.addData("  Left Stick", "Manual control");
        telemetry.update();
    }

    @Override
    public void stop() {
        stopTurret();
        if (limelight != null) {
            limelight.stop();
        }
    }
}