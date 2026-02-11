package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Position Tuner", group = "TeleOp")
public class ArmPositionTuner extends LinearOpMode {

    private Servo armLeft, armBack, armRight;

    @Override
    public void runOpMode() {

        armLeft  = hardwareMap.get(Servo.class, "al");
        armBack  = hardwareMap.get(Servo.class, "ab");
        armRight = hardwareMap.get(Servo.class, "ar");

        // initialize positions
        armLeft.setPosition(1);
        armBack.setPosition(0.95);
        armRight.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            // --------- LEFT ARM ---------
            if (gamepad2.dpad_up) {
                armLeft.setPosition(Math.min(armLeft.getPosition() + 0.1, 1.0));
            }
            if (gamepad2.dpad_down) {
                armLeft.setPosition(Math.max(armLeft.getPosition() - 0.1, 0.0));
            }

            // --------- BACK ARM ---------
            if (gamepad2.dpad_left) {
                armBack.setPosition(Math.min(armBack.getPosition() + 0.1, 1.0));
            }
            if (gamepad2.dpad_right) {
                armBack.setPosition(Math.max(armBack.getPosition() - 0.1, 0.0));
            }

            // --------- RIGHT ARM ---------
            if (gamepad2.a) {
                armRight.setPosition(Math.min(armRight.getPosition() + 0.1, 1.0));
            }
            if (gamepad2.b) {
                armRight.setPosition(Math.max(armRight.getPosition() - 0.1, 0.0));
            }

            // --------- TELEMETRY ---------
            telemetry.addData("Left Arm", armLeft.getPosition());
            telemetry.addData("Back Arm", armBack.getPosition());
            telemetry.addData("Right Arm", armRight.getPosition());
            telemetry.update();

            sleep(150); // small delay so the increments are manageable
        }
    }
}
