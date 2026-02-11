package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Single Motor Tester", group = "Test")
public class SingleMotorTester extends LinearOpMode {

    DcMotor leftFront, leftRear, rightFront, rightRear;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotor.class, "fl");
        leftRear   = hardwareMap.get(DcMotor.class, "bl");
        rightFront = hardwareMap.get(DcMotor.class, "fr");
        rightRear  = hardwareMap.get(DcMotor.class, "br");

        waitForStart();
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {

            // stop all motors first (VERY IMPORTANT)
            leftFront.setPower(0);
            leftRear.setPower(0);
            rightFront.setPower(0);
            rightRear.setPower(0);

            if (gamepad1.a) {
                leftFront.setPower(1.0);
            }
            else if (gamepad1.b) {
                leftRear.setPower(1.0);
            }
            else if (gamepad1.x) {
                rightFront.setPower(1.0);
            }
            else if (gamepad1.y) {
                rightRear.setPower(1.0);
            }
        }
    }
}
