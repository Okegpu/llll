package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@Autonomous(name = "AutoBlue", group = "Test")
public class AutoBlue extends LinearOpMode {

    // ================= DRIVE =================
    private DcMotor fLMotor, fRMotor, bLMotor, bRMotor;

    // ================= MECHANISMS =================
    private DcMotor shootMotor;
    private DcMotor inMotor;
    private DcMotor pushMotor;
    private CRServo spinServo;

    // ================= LIMELIGHT =================
    private Limelight3A limelight;

    // ================= AIM TUNING =================
    private static final double AIM_KP  = 0.025;
    private static final double AIM_MIN = 0.08;
    private static final double AIM_MAX = 0.35;
    private static final double AIM_TOL = 0.7;

    // Camera is 9° RIGHT of shooter
    private static final double CAM_TX_OFFSET = -9.0;

    @Override
    public void runOpMode() {

        // ================= HARDWARE =================
        fLMotor = hardwareMap.get(DcMotor.class, "front_left");
        fRMotor = hardwareMap.get(DcMotor.class, "front_right");
        bLMotor = hardwareMap.get(DcMotor.class, "back_left");
        bRMotor = hardwareMap.get(DcMotor.class, "back_right");

        shootMotor = hardwareMap.get(DcMotor.class, "top_motor");
        inMotor    = hardwareMap.get(DcMotor.class, "in_motor");
        pushMotor  = hardwareMap.get(DcMotor.class, "p_motor");
        spinServo  = hardwareMap.get(CRServo.class, "spin_servo");

        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ================= LIMELIGHT =================
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag
        limelight.start();

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        autoAimAndShoot();
    }

    // ====================================================
    // AUTO AIM + SHOOT (LEFT TURN FIXED)
    // ====================================================
    private void autoAimAndShoot() {

        int lostFrames = 0;
        long lockStartTime = -1;
        long timeout = System.currentTimeMillis() + 5000;

        while (opModeIsActive() && System.currentTimeMillis() < timeout) {

            LLResult r = limelight.getLatestResult();

            // ---------- TARGET LOST ----------
            if (r == null || !r.isValid()) {
                lostFrames++;
                if (lostFrames > 5) {
                    mecanum(0, 0, 0.12);
                    telemetry.addLine("Searching...");
                }
                telemetry.update();
                continue;
            }

            lostFrames = 0;

            double tx = r.getTx();

            // Fixed offset only
            double correctedTx = tx + CAM_TX_OFFSET;
            double error = correctedTx;

            // 🔁 TURN LEFT FIX (SIGN FLIPPED HERE ONLY)
            double turn = -error * AIM_KP;

            if (Math.abs(error) < 5)
                turn *= 0.4;

            if (Math.abs(error) > 2 && Math.abs(turn) < AIM_MIN)
                turn = Math.copySign(AIM_MIN, turn);

            turn = Math.max(-AIM_MAX, Math.min(AIM_MAX, turn));

            mecanum(0, 0, turn);

            telemetry.addData("tx", tx);
            telemetry.addData("correctedTx", correctedTx);
            telemetry.addData("turn", turn);
            telemetry.update();

            // ---------- STABLE LOCK ----------
            if (Math.abs(error) < AIM_TOL) {
                if (lockStartTime < 0)
                    lockStartTime = System.currentTimeMillis();

                if (System.currentTimeMillis() - lockStartTime > 250) {
                    mecanum(0, 0, 0);
                    break;
                }
            } else {
                lockStartTime = -1;
            }
        }

        // ================= SHOOT =================
        telemetry.addLine("LOCKED — SHOOTING");
        telemetry.update();

        shootMotor.setPower(1.0);
        inMotor.setPower(-1.0);
        spinServo.setPower(1.0);

        sleep(700);

        long shootEnd = System.currentTimeMillis() + 3000;
        while (opModeIsActive() && System.currentTimeMillis() < shootEnd) {
            pushMotor.setPower(-1.0);
            idle();
        }

        pushMotor.setPower(0);
        shootMotor.setPower(0);
        inMotor.setPower(0);
        spinServo.setPower(0);
    }

    // ================= DRIVE =================
    private void mecanum(double x, double y, double rx) {
        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fLMotor.setPower(fl / max);
        fRMotor.setPower(fr / max);
        bLMotor.setPower(bl / max);
        bRMotor.setPower(br / max);
    }
}
