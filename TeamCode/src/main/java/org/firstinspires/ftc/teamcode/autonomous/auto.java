package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BackUp + Shoot MEDIUM left", group = "Auto")
public class auto extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor fL, fR, bL, bR;

    // ================= SHOOTER =================
    private DcMotorEx shooter1, shooter2;

    // ================= INDEXER =================
    private Servo indexer, kicker;

    // ===== DRIVE TUNING =====
    private static final double BACK_POWER = -0.5;
    private static final long BACK_TIME_MS = 1350;

    // ===== SHOOTER (MEDIUM) =====
    private static final double VEL_MEDIUM = 1500;

    // ===== INDEXER POSITIONS (FROM YOUR TELEOP) =====
    private static final double[] SHOOT_POS = {0.50, 0.68, 0.89};
    private static final double INDEXER_HOME = 0.20;   // slot1 / intake home

    // ===== KICKER =====
    private static final double KICKER_DOWN = 1.00;
    private static final double KICKER_UP   = 0.35;

    // ===== TIMING =====
    private static final long SPINUP_MS = 800;
    private static final long INDEXER_MOVE_MS = 300;
    private static final long INDEXER_SETTLE_MS = 180;

    private static final long KICKER_UP_MS = 350;
    private static final long KICKER_HOLD_MS = 160;
    private static final long KICKER_DOWN_MS = 450;

    @Override
    public void runOpMode() {

        // ================= HW MAP =================
        fL = hardwareMap.get(DcMotor.class, "front_left");
        fR = hardwareMap.get(DcMotor.class, "front_right");
        bL = hardwareMap.get(DcMotor.class, "back_left");
        bR = hardwareMap.get(DcMotor.class, "back_right");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer = hardwareMap.get(Servo.class, "indexer");
        kicker  = hardwareMap.get(Servo.class, "kick");

        // ===== "ZERO" INDEXER + SAFE KICKER BEFORE START =====
        kicker.setPosition(KICKER_DOWN);
        indexer.setPosition(INDEXER_HOME);
        sleep(300);

        telemetry.addLine("READY: Back up, then shoot MEDIUM");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ================= 1) DRIVE BACKWARD =================
        drive(0, BACK_POWER, 0);
        sleep(BACK_TIME_MS);
        drive(0, 0, 0);

        // ================= 2) SPIN UP SHOOTER =================
        shooter1.setVelocity(VEL_MEDIUM);
        shooter2.setVelocity(VEL_MEDIUM);
        sleep(SPINUP_MS);

        // Pre-stage the first shot so it isn't "too far"
        indexer.setPosition(SHOOT_POS[0]);
        sleep(350);
        kicker.setPosition(KICKER_DOWN);
        sleep(150);

        // ================= 3) SHOOT ALL 3 =================
        for (int slot = 0; slot < 3 && opModeIsActive(); slot++) {
            shoot(slot);
        }

        // ================= STOP + STRAFE LEFT =================
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        kicker.setPosition(KICKER_DOWN);

        drive(-0.6, 0.0, 0.0);
        sleep(2000);
        drive(0, 0, 0);
    }

    // ================= HELPERS =================
    private void shoot(int slot) {
        indexer.setPosition(SHOOT_POS[slot]);

        // extra settle on first shot only
        long extra = (slot == 0) ? 250 : 0;
        sleep(INDEXER_MOVE_MS + INDEXER_SETTLE_MS + extra);

        kicker.setPosition(KICKER_UP);
        sleep(KICKER_UP_MS);

        sleep(KICKER_HOLD_MS);

        kicker.setPosition(KICKER_DOWN);
        sleep(KICKER_DOWN_MS);
    }

    private void drive(double x, double y, double rx) {
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
