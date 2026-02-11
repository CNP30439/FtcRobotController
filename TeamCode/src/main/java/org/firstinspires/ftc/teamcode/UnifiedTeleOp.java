package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Unified TeleOp", group = "TeleOp")
public class UnifiedTeleOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx s1, s2;
    private DcMotor fi;
    private Servo ha;
    private CRServo tl, tr;
    private Servo armLeft, armBack, armRight;
    private ColorSensor leftCs, backCs, rightCs;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;
    private VoltageSensor voltageSensor;

    private double hoodPos = 0.7;
    private final double HOOD_MIN = 0;
    private final double HOOD_MAX = 1;
    private final double HOOD_STEP = 0.1;
    private GoBildaPrismDriver prism;
    private int currentArtboardState = 0; // Tracks current ball count (0-3)
    private int previousBallCount = 0;

    // ---- TURRET TRACKING (from TurretYawTuneTest) ----
    private double yawOffset = 0.0;
    private static final double TURRET_KP = 0.015;
    private static final double TURRET_DEADZONE = 1.5;
    private static final double TURRET_MIN = 0.05;
    private static final double TURRET_MAX = 0.3;

    private boolean manualTurretControl = false;
    private static final double MANUAL_TURRET_POWER = 0.3;

    private static final double NOMINAL_VOLTAGE = 12.0;
    private double voltageCompensation = 1.0;

    long shooterStartTime = 0;
    boolean shooterActive = false;
    boolean shooterSpunUp = false;
    boolean autoRamping = false;
    long lastDetectionTime = 0;
    boolean wasDetecting = false;

    final long SPINUP_TIME_MS = 1450;
    final long BOOST_TIME_MS = 450;
    final long KEEP_ALIVE_TIME_MS = 5000;

    final int COLOR_DETECT_THRESHOLD = 100;

    // Updated tuned shooter velocities from Blue Purple-Green-Purple
    private final double HIGH_VELOCITY = 950;
    private final double LOW_VELOCITY = 810;

    // Priority order will be set based on selected mode
    private String[] priorityOrder;
    private List<String> armsToLift = new ArrayList<>();
    private int sequenceStep = 0;
    private boolean sequenceActive = false;
    private long armMoveStartTime = 0;
    private final long ARM_MOVE_DELAY = 500; // Updated from Blue Purple-Green-Purple
    private boolean armLifted = false;

    // ---- FOURTH COLOR SENSOR (4b) ----
    private ColorSensor fourthCs;
    private long fourthSensorDetectionStart = 0;
    private boolean fourthSensorDetecting = false;
    private boolean spittingOut = false;
    private static final long DETECTION_TIME_THRESHOLD = 2000; // 2 seconds in milliseconds

    boolean lastX = false, lastY = false;

    // AprilTag ID will be set based on alliance
    private int targetTagId;

    // Rotation multiplier for turning
    private static final double ROTATION_MULTIPLIER = 1.5;

    // Mode selection variables
    private int selectedModeIndex = 0;
    private String[] modeNames = {
            "Blue Purple-Green-Purple",
            "Blue Purple-Purple-Green",
            "Blue Green-Purple-Purple",
            "Red Purple-Green-Purple",
            "Red Purple-Purple-Green",
            "Red Green-Purple-Purple"
    };

    @Override
    public void runOpMode() {

        // ---- MODE SELECTION ----
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        telemetry.addLine("=== MODE SELECTION ===");
        telemetry.addLine("Use DPAD UP/DOWN to select mode");
        telemetry.addLine("Press START when ready");
        telemetry.addLine("");

        while (!isStarted() && !isStopRequested()) {
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            // Detect rising edge for dpad up
            if (currentDpadUp && !lastDpadUp) {
                selectedModeIndex--;
                if (selectedModeIndex < 0) selectedModeIndex = modeNames.length - 1;
            }

            // Detect rising edge for dpad down
            if (currentDpadDown && !lastDpadDown) {
                selectedModeIndex++;
                if (selectedModeIndex >= modeNames.length) selectedModeIndex = 0;
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            // Display all modes with selection indicator
            for (int i = 0; i < modeNames.length; i++) {
                if (i == selectedModeIndex) {
                    telemetry.addLine(">>> " + modeNames[i] + " <<<");
                } else {
                    telemetry.addLine("    " + modeNames[i]);
                }
            }
            telemetry.update();
        }

        // Configure mode based on selection
        configureModeSettings(selectedModeIndex);

        // ---- HARDWARE INITIALIZATION ----
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        s1 = hardwareMap.get(DcMotorEx.class, "s1");
        s2 = hardwareMap.get(DcMotorEx.class, "s2");
        fi = hardwareMap.get(DcMotor.class, "fi");

        ha = hardwareMap.get(Servo.class, "ha");
        tl = hardwareMap.get(CRServo.class, "tl");
        tr = hardwareMap.get(CRServo.class, "tr");

        armLeft  = hardwareMap.get(Servo.class, "al");
        armBack  = hardwareMap.get(Servo.class, "ab");
        armRight = hardwareMap.get(Servo.class, "ar");

        leftCs  = hardwareMap.get(ColorSensor.class, "leftCs");
        backCs  = hardwareMap.get(ColorSensor.class, "backCs");
        rightCs = hardwareMap.get(ColorSensor.class, "rightCs");
        fourthCs = hardwareMap.get(ColorSensor.class, "4b");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        // Initialize Prism LED driver
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (Limelight3A ll : hardwareMap.getAll(Limelight3A.class)) {
            limelight = ll;
            break;
        }
        if (limelight != null) {
            limelight.pipelineSwitch(1);
            limelight.start();
        }

        // Motor directions from Blue Purple-Green-Purple (updated pattern)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes for max speed
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.REVERSE);
        fi.setDirection(DcMotor.Direction.REVERSE);

        // Configure shooter motors with tuned PIDF values
        s1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(550, 0, 0, 15.9);
        s1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);
        s2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        ha.setPosition(0.4);
        armLeft.setPosition(.9);
        armBack.setPosition(1);
        armRight.setPosition(1);

        // Initialize LEDs to Artboard 1 (not full)
        // Initialize LEDs to Artboard 1 (no balls)
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        currentArtboardState = 0;

        waitForStart();

        while (opModeIsActive()) {

            updateVoltageCompensation();

            // ---- DRIVE ----
            double y  = -gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x * ROTATION_MULTIPLIER;

            double fl = y + x + rx;
            double fr = y - x - rx;
            double bl = y - x + rx;
            double br = y + x - rx;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            // No voltage compensation on drive motors
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // ---- HOOD ----
            if (gamepad1.a) {
                hoodPos = 0.7;
                ha.setPosition(hoodPos);
            } else if (gamepad1.b) {
                hoodPos = 0.5;
                ha.setPosition(hoodPos);
            }
            if (gamepad1.x && !lastX) hoodPos += HOOD_STEP;
            if (gamepad1.y && !lastY) hoodPos -= HOOD_STEP;
            hoodPos = Math.max(HOOD_MIN, Math.min(HOOD_MAX, hoodPos));
            ha.setPosition(hoodPos);
            lastX = gamepad1.x;
            lastY = gamepad1.y;

            // ---- TURRET TRACKING (from TurretYawTuneTest) ----
// Adjust offset live
            if (gamepad1.dpad_up) yawOffset -= 0.05;
            if (gamepad1.dpad_down) yawOffset += 0.05;

// Manual turret control on gamepad2
            boolean manualLeft = gamepad2.dpad_left;
            boolean manualRight = gamepad2.dpad_right;

            if (manualLeft || manualRight) {
                // Manual control active
                manualTurretControl = true;

                if (manualLeft) {
                    tl.setPower(-MANUAL_TURRET_POWER);
                    tr.setPower(-MANUAL_TURRET_POWER);
                } else if (manualRight) {
                    tl.setPower(MANUAL_TURRET_POWER);
                    tr.setPower(MANUAL_TURRET_POWER);
                }
            } else {
                // No manual input - resume automatic tracking
                manualTurretControl = false;

                if (limelight != null) {
                    LLResult r = limelight.getLatestResult();

                    if (r != null && r.isValid()) {
                        boolean tagFound = false;

                        for (LLResultTypes.FiducialResult t : r.getFiducialResults()) {
                            if (t.getFiducialId() == targetTagId) {
                                tagFound = true;

                                double tx = t.getTargetXDegrees() + yawOffset;

                                double power = 0;
                                if (Math.abs(tx) > TURRET_DEADZONE) {
                                    power = Math.signum(tx) *
                                            Math.min(TURRET_MAX, Math.max(TURRET_MIN, Math.abs(tx * TURRET_KP)));
                                }

                                tl.setPower(power);
                                tr.setPower(power);
                                break;
                            }
                        }

                        if (!tagFound) {
                            tl.setPower(0);
                            tr.setPower(0);
                        }
                    } else {
                        tl.setPower(0);
                        tr.setPower(0);
                    }
                } else {
                    tl.setPower(0);
                    tr.setPower(0);
                }
            }

            // ---- INTAKE ----
// Check fourth sensor (4b) for spit-out logic
            String fourthColor = detectBallColor(fourthCs);
            boolean fourthDetectingBall = !fourthColor.equals("None");

// Count detected balls in main sensors
            int mainBallCount = 0;
            if (!detectBallColor(leftCs).equals("None")) mainBallCount++;
            if (!detectBallColor(backCs).equals("None")) mainBallCount++;
            if (!detectBallColor(rightCs).equals("None")) mainBallCount++;

// Handle detection timing for fourth sensor
            if (fourthDetectingBall && mainBallCount == 3 && (fourthColor.equals("Purple") || fourthColor.equals("Green"))) {
                if (!fourthSensorDetecting) {
                    // Ball just detected, start timer
                    fourthSensorDetectionStart = System.currentTimeMillis();
                    fourthSensorDetecting = true;
                } else {
                    // Ball still detected, check if time threshold passed
                    long detectionDuration = System.currentTimeMillis() - fourthSensorDetectionStart;
                    if (detectionDuration >= DETECTION_TIME_THRESHOLD) {
                        spittingOut = true;
                    }
                }
            } else {
                // No ball detected or not full or wrong color
                fourthSensorDetecting = false;
                fourthSensorDetectionStart = 0;
                if (spittingOut && !fourthDetectingBall) {
                    // Ball has been spit out successfully
                    spittingOut = false;
                }
            }

// Control intake motor
            if (spittingOut) {
                // Spit out the ball (reverse)
                fi.setPower(1);
            } else if (gamepad2.x) {
                fi.setPower(1);
            } else if (gamepad2.y) {
                fi.setPower(-1);
            } else {
                fi.setPower(0);
            }
            // ---- COLOR DETECTION ----
            String leftColor  = detectBallColor(leftCs);
            String backColor  = detectBallColor(backCs);
            String rightColor = detectBallColor(rightCs);

// Count detected balls
            int ballCount = 0;
            if (!leftColor.equals("None")) ballCount++;
            if (!backColor.equals("None")) ballCount++;
            if (!rightColor.equals("None")) ballCount++;

// Update Prism LEDs based on ball count
            updatePrismLEDs(ballCount);

            // ---- ARM SEQUENCE ----
            if (gamepad2.dpad_up && !sequenceActive) {
                buildArmSequence();
                if (!armsToLift.isEmpty()) {
                    sequenceActive = true;
                    sequenceStep = 0;
                    armLifted = false;
                    armMoveStartTime = System.currentTimeMillis();
                }
            }

            if (sequenceActive) processArmSequence();

            // ---- SHOOTER ----
            boolean detected = detectObject();
            if (detected) {
                lastDetectionTime = System.currentTimeMillis();
                wasDetecting = true;
            }

            long dt = System.currentTimeMillis() - lastDetectionTime;
            boolean keepAlive = wasDetecting && dt < KEEP_ALIVE_TIME_MS;

            boolean shooterRequested = false;
            double targetVelocity = 0;
            boolean lowPowerMode = false;

            if (gamepad2.a) {
                shooterRequested = true;
                targetVelocity = HIGH_VELOCITY;
            } else if (gamepad2.b) {
                shooterRequested = true;
                targetVelocity = LOW_VELOCITY;
                lowPowerMode = true;
            } else if (detected || keepAlive) {
                shooterRequested = true;
                targetVelocity = HIGH_VELOCITY;
            }

// Auto-adjust hood angle based on low power mode
            if (lowPowerMode) {
                hoodPos = 0.7;
                ha.setPosition(hoodPos);
            } else if (!gamepad2.b) {
                // Return to 0.5 when not holding B (unless manually adjusted with X/Y)
                if (!gamepad1.x && !gamepad1.y && hoodPos == 0.7) {
                    hoodPos = 0.5;
                    ha.setPosition(hoodPos);
                }
            }

            if (shooterRequested) {
                if (!shooterActive) {
                    shooterStartTime = System.currentTimeMillis();
                    shooterActive = true;
                    shooterSpunUp = false;
                }

                long elapsed = System.currentTimeMillis() - shooterStartTime;

                if (!shooterSpunUp && elapsed > SPINUP_TIME_MS) {
                    shooterSpunUp = true;
                    gamepad2.rumble(1, 1, 500);
                }

                s1.setVelocity(targetVelocity);
                s2.setVelocity(targetVelocity);
            } else {
                shooterActive = false;
                shooterSpunUp = false;
                s1.setVelocity(0);
                s2.setVelocity(0);
            }

            // ---- TELEMETRY ----
            telemetry.addData("Mode", modeNames[selectedModeIndex]);
            telemetry.addData("LEFT RGB",  "R:%d G:%d B:%d A:%d",
                    leftCs.red(), leftCs.green(), leftCs.blue(), leftCs.alpha());
            telemetry.addData("BACK RGB",  "R:%d G:%d B:%d A:%d",
                    backCs.red(), backCs.green(), backCs.blue(), backCs.alpha());
            telemetry.addData("RIGHT RGB", "R:%d G:%d B:%d A:%d",
                    rightCs.red(), rightCs.green(), rightCs.blue(), rightCs.alpha());

            telemetry.addData("Detected Left", leftColor);
            telemetry.addData("Detected Back", backColor);
            telemetry.addData("Detected Right", rightColor);
            telemetry.addData("Sequence", armsToLift.toString());
            telemetry.addData("Shooter Velocity", targetVelocity);
            telemetry.addData("Shooter Active", shooterActive);
            telemetry.addData("Shooter Spun Up", shooterSpunUp);
            telemetry.addData("Hood", hoodPos);
            telemetry.addData("Ball Count", ballCount);
            telemetry.addData("LED Artboard", "Artboard " + (ballCount == 0 ? 1 : ballCount == 3 ? 0 : ballCount + 1) + " (" + ballCount + " balls)");
            telemetry.update();
        }
    }

    // ---------- MODE CONFIGURATION ----------

    private void configureModeSettings(int modeIndex) {
        switch (modeIndex) {
            case 0: // Blue Purple-Green-Purple
                priorityOrder = new String[]{"Purple", "Green", "Purple"};
                targetTagId = 20;
                break;
            case 1: // Blue Purple-Purple-Green
                priorityOrder = new String[]{"Purple", "Purple", "Green"};
                targetTagId = 20;
                break;
            case 2: // Blue Green-Purple-Purple
                priorityOrder = new String[]{"Green", "Purple", "Purple"};
                targetTagId = 20;
                break;
            case 3: // Red Purple-Green-Purple
                priorityOrder = new String[]{"Purple", "Green", "Purple"};
                targetTagId = 24;
                break;
            case 4: // Red Purple-Purple-Green
                priorityOrder = new String[]{"Purple", "Purple", "Green"};
                targetTagId = 24;
                break;
            case 5: // Red Green-Purple-Purple
                priorityOrder = new String[]{"Green", "Purple", "Purple"};
                targetTagId = 24;
                break;
        }
    }

    // ---------- HELPERS ----------

    private void updateVoltageCompensation() {
        voltageCompensation = NOMINAL_VOLTAGE / voltageSensor.getVoltage();
    }

    private double applyVoltageComp(double power) {
        return Math.max(-1, Math.min(1, power * voltageCompensation));
    }

    private boolean detectObject() {
        return leftCs.alpha() > COLOR_DETECT_THRESHOLD ||
                backCs.alpha() > COLOR_DETECT_THRESHOLD ||
                rightCs.alpha() > COLOR_DETECT_THRESHOLD;
    }

    // 🔥 UNKNOWN = PURPLE
    private String detectBallColor(ColorSensor s) {
        if (s.alpha() < COLOR_DETECT_THRESHOLD) return "None";
        if (s.green() > s.red() && s.green() > s.blue()) return "Green";
        return "Purple";
    }

    private void buildArmSequence() {
        armsToLift.clear();

        String l = detectBallColor(leftCs);
        String b = detectBallColor(backCs);
        String r = detectBallColor(rightCs);

        List<String> avail = new ArrayList<>();
        if (!l.equals("None")) avail.add("left:" + l);
        if (!b.equals("None")) avail.add("back:" + b);
        if (!r.equals("None")) avail.add("right:" + r);

        for (String p : priorityOrder) {
            for (int i = 0; i < avail.size(); i++) {
                if (avail.get(i).endsWith(p)) {
                    armsToLift.add(avail.get(i).split(":")[0]);
                    avail.remove(i);
                    break;
                }
            }
        }
        for (String a : avail) armsToLift.add(a.split(":")[0]);
    }
    private void updatePrismLEDs(int ballCount) {
        // Only update if ball count changed (prevents unnecessary commands)
        if (ballCount != currentArtboardState) {
            switch (ballCount) {
                case 0:
                    // No balls - Artboard 1 (orange)
                    prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
                    break;
                case 1:
                    // 1 ball - Artboard 2
                    prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_2);
                    break;
                case 2:
                    // 2 balls - Artboard 3
                    prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_3);
                    break;
                case 3:
                    // 3 balls - Artboard 0 (rainbow)
                    prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
                    break;
            }
            currentArtboardState = ballCount;
        }
    }
    private void liftArm(String a) {
        if (a.equals("left")) armLeft.setPosition(0.3);
        if (a.equals("back")) armBack.setPosition(0.3);
        if (a.equals("right")) armRight.setPosition(0.3);
    }

    private void lowerArm(String a) {
        if (a.equals("left")) armLeft.setPosition(.9);
        if (a.equals("back")) armBack.setPosition(1);
        if (a.equals("right")) armRight.setPosition(1);
    }

    private void processArmSequence() {
        long t = System.currentTimeMillis() - armMoveStartTime;
        if (sequenceStep >= armsToLift.size()) {
            sequenceActive = false;
            armsToLift.clear();
            return;
        }
        String arm = armsToLift.get(sequenceStep);
        if (!armLifted && t > ARM_MOVE_DELAY) {
            liftArm(arm);
            armLifted = true;
            armMoveStartTime = System.currentTimeMillis();
        } else if (armLifted && t > ARM_MOVE_DELAY) {
            lowerArm(arm);
            sequenceStep++;
            armLifted = false;
            armMoveStartTime = System.currentTimeMillis();
        }
    }


    private void configurePinpoint() {
        pinpoint.setOffsets(0, 0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}
