package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "StraightThenStrafeleft", group = "Auto")
public class Autoleft extends LinearOpMode {

    private DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() {

        // Motors
        fL = hardwareMap.get(DcMotor.class, "front_left");
        fR = hardwareMap.get(DcMotor.class, "front_right");
        bL = hardwareMap.get(DcMotor.class, "back_left");
        bR = hardwareMap.get(DcMotor.class, "back_right");

        // Reverse left side (typical mecanum)
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // =====================
        // DRIVE FORWARD (10 sec)
        // =====================
        sleep(10_000);
        setDrive(0.0, 0.6, 0.0);
        sleep(2_000);

        // =====================
        // STRAFE LEFT (2 sec)
        // =====================
        setDrive(-0.6, 0.0, 0.0);
        sleep(1_000);

        // STOP
        setDrive(0, 0, 0);
    }

    // Mecanum helper
    private void setDrive(double x, double y, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fL.setPower(fl / max);
        fR.setPower(fr / max);
        bL.setPower(bl / max);
        bR.setPower(br / max);
    }
}
