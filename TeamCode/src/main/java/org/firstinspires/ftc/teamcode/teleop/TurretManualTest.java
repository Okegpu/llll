package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TURRET MANUAL TEST", group="TEST")
public class TurretManualTest extends LinearOpMode {

    private DcMotorEx turret;

    @Override
    public void runOpMode() {

        // MUST match name in Robot Config exactly
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("READY");
        telemetry.addLine("GP1 DPAD LEFT/RIGHT = move turret");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double power = 0.4; // speed

            if (gamepad1.dpad_right) {
                turret.setPower(power);
            }
            else if (gamepad1.dpad_left) {
                turret.setPower(-power);
            }
            else {
                turret.setPower(0);
            }

            telemetry.addData("Turret Power",
                    gamepad1.dpad_right ? "+0.4" :
                            gamepad1.dpad_left ? "-0.4" : "0");

            telemetry.addData("Encoder Ticks",
                    turret.getCurrentPosition());

            telemetry.update();
        }
    }
}
