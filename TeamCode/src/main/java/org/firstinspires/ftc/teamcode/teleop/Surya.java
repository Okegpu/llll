package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Surya: Simple Hood + A Button", group="Drive")
public class Surya extends LinearOpMode {

    // =========================================================
    // PHYSICAL MEASUREMENTS
    // =========================================================
    private static final double GOAL_HEIGHT_IN = 38.75;
    private static final double CAMERA_HEIGHT_IN = 12.5;
    private static final double WHEEL_DIAMETER_CM = 10.0;

    // PHYSICS CONSTANTS
    private static final double GRAVITY = 9.81;
    private static final double INCH_TO_METER = 0.0254;
    private static final double DELTA_H_M = (GOAL_HEIGHT_IN - CAMERA_HEIGHT_IN) * INCH_TO_METER;
    private static final double CAMERA_MOUNT_ANGLE_DEG = 25.0;

    // =========================================================
    // HOOD SETTINGS (TUNE THESE!)
    // =========================================================
    private static final double THRESHOLD_METERS = 2.0; // 6.5 feet

    // CLOSE (High Arc)
    private static final double HOOD_CLOSE_POS = 0.65;
    private static final double HOOD_CLOSE_DEG = 60.0;

    // FAR (Power Shot)
    private static final double HOOD_FAR_POS   = 0.45;
    private static final double HOOD_FAR_DEG   = 40.0;

    // =======================
    // HARDWARE
    // =======================
    private DcMotor fLMotor, fRMotor, bLMotor, bRMotor;
    private DcMotor evil;
    private DcMotorEx turret, shooter1, shooter2;
    private Servo indexer, kicker, hood;
    private ColorSensor color;
    private DistanceSensor distance;
    private Limelight3A limelight;

    // =======================
    // VARIABLES
    // =======================
    private static final double DB = 0.06;
    private static final double STRAFE_SCALE = 1.10;

    private boolean evilOn = false;
    private boolean lastLB = false;
    private static final double INTAKE_POWER  =  1.0;
    private static final double OUTTAKE_POWER = -1.0;

    private double currentDistMeters = 1.5;
    private boolean targetVisible = false;

    private static final double TICKS_PER_TURRET_REV = 383.6 * (100.0/54.0);
    private double lastTx = 0;
    private long lastTxMs = 0;

    private final double[] SLOT_POS  = {0.20, 0.38, 0.59};
    private final double[] SHOOT_POS = {0.50, 0.68, 0.89};
    private static final double KICKER_DOWN = 1.00;
    private static final double KICKER_UP   = 0.35;

    // Inputs
    private int manualShootSlot = 0;
    private boolean lastDpadRight = false;
    private boolean lastA = false; // Added lastA state
    private int rpmOffset = 0;
    private boolean lastRB=false, lastRT=false;

    public enum BallColor { EMPTY, GREEN, PURPLE }
    private final BallColor[] slotColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private final boolean[] slotFull = {false,false,false};
    private int nextFillSlot = 0;
    private boolean armed = true;
    private boolean intakeEnabled = true;

    private ShootState shootState = ShootState.IDLE;
    private enum ShootState { IDLE, INDEX_TO_SHOOT, KICK_UP, HOLD, KICK_DOWN }
    private long shootT0=0;
    private boolean indexerBusy=false;
    private long indexerBusyUntilMs=0;
    private int shootingSlot = -1;
    private boolean clearSlotMemoryOnFinish = true;

    private enum ResetState { NONE, KICK_DOWN, INDEX_TO_NEXT }
    private ResetState resetState = ResetState.NONE;
    private long resetT0=0;

    private BallColor lastDet=BallColor.EMPTY;
    private int detCount=0;
    private long lastStoreMs=0;

    @Override
    public void runOpMode() {
        // HARDWARE MAP
        fLMotor = hardwareMap.get(DcMotor.class,"front_left");
        fRMotor = hardwareMap.get(DcMotor.class,"front_right");
        bLMotor = hardwareMap.get(DcMotor.class,"back_left");
        bRMotor = hardwareMap.get(DcMotor.class,"back_right");

        fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        evil = hardwareMap.get(DcMotor.class,"evil");
        evil.setDirection(DcMotorSimple.Direction.REVERSE);

        indexer = hardwareMap.get(Servo.class,"indexer");
        kicker  = hardwareMap.get(Servo.class,"kick");
        hood    = hardwareMap.get(Servo.class, "angle"); // Mapped to "angle"

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        color = hardwareMap.get(ColorSensor.class,"color");
        distance = hardwareMap.get(DistanceSensor.class,"color");
        color.enableLed(true);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        kicker.setPosition(KICKER_DOWN);
        sleep(250);
        clearAll();
        recalcNextFillSlot();
        goToSlot(nextFillSlot);

        telemetry.addLine("READY: SIMPLE HOOD + A BUTTON");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();
            boolean servoBusy = indexerBusy || (shootState != ShootState.IDLE) || (resetState != ResetState.NONE);

            // 1. DRIVE
            double x = gamepad1.left_stick_x * STRAFE_SCALE;
            double y = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;
            if(Math.abs(x)<DB)x=0; if(Math.abs(y)<DB)y=0; if(Math.abs(rx)<DB)rx=0;

            double fl=y+x+rx; double fr=y-x-rx; double bl=y-x+rx; double br=y+x-rx;
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fLMotor.setPower(fl/max); fRMotor.setPower(fr/max); bLMotor.setPower(bl/max); bRMotor.setPower(br/max);

            // 2. INTAKE
            boolean outtakeHeld = (gamepad1.left_trigger > 0.6);
            if(gamepad1.left_bumper && !lastLB) evilOn = !evilOn;
            lastLB = gamepad1.left_bumper;

            if(outtakeHeld) {
                evil.setPower(OUTTAKE_POWER);
                intakeEnabled = false; armed = false;
            } else {
                evil.setPower(evilOn ? INTAKE_POWER : 0.0);
                if(evilOn) intakeEnabled = true;
            }

            // 3. LIMELIGHT
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid() && result.getFiducialResults().size() > 0) {
                targetVisible = true;
                double tx = result.getTx();
                double ty = result.getTy();

                double angleRad = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
                currentDistMeters = DELTA_H_M / Math.tan(angleRad);

                if(Math.abs(tx) > 1.0) {
                    double p = 0.02 * tx;
                    turret.setPower(-p);
                } else turret.setPower(0);

            } else {
                targetVisible = false;
                turret.setPower(0);
            }

            // 4. PHYSICS & HOOD LOGIC
            if(gamepad1.right_bumper && !lastRB) rpmOffset += 50;
            if((gamepad1.right_trigger > 0.5) && !lastRT) rpmOffset -= 50;
            lastRB = gamepad1.right_bumper; lastRT = (gamepad1.right_trigger > 0.5);

            solvePhysicsShot(currentDistMeters);

            // 5. SPINDEXER UPDATE
            updateMechanisms(now);

            // INPUTS
            boolean dRight = gamepad1.dpad_right;
            if(dRight && !lastDpadRight && shootState == ShootState.IDLE) {
                startShoot(manualShootSlot, false);
                manualShootSlot = (manualShootSlot + 1) % 3;
            }
            lastDpadRight = dRight;

            // X = FULL RESET (Clears Memory)
            if(gamepad1.x && resetState == ResetState.NONE) startReset();

            // A = RETURN TO INTAKE (Keeps Memory)
            boolean aPressed = gamepad1.a && !lastA;
            if(aPressed && !servoBusy) {
                intakeEnabled = true;
                armed = true;
                recalcNextFillSlot();
                kicker.setPosition(KICKER_DOWN);
                goToSlot(nextFillSlot);
            }
            lastA = gamepad1.a;

            // SMART SHOOT (B / Y)
            if(shootState == ShootState.IDLE) {
                if(gamepad1.b) {
                    int i = findSlot(BallColor.GREEN);
                    if(i!=-1) startShoot(i, true);
                }
                if(gamepad1.y) {
                    int i = findSlot(BallColor.PURPLE);
                    if(i!=-1) startShoot(i, true);
                }
            }

            // AUTO STORE
            double dCm = distance.getDistance(DistanceUnit.CM);
            if(intakeEnabled && armed && !isFull() && dCm < 6.0) {
                int r = color.red(); int g = color.green(); int b = color.blue();
                BallColor detected = (g > r && g > b) ? BallColor.GREEN : (r+b > g*1.5) ? BallColor.PURPLE : BallColor.EMPTY;

                if(detected != BallColor.EMPTY) {
                    if(detected == lastDet) detCount++; else { detCount=1; lastDet=detected; }
                    if(detCount > 3 && (now - lastStoreMs > 350)) {
                        storeBall(detected);
                        lastStoreMs = now; armed = false;
                    }
                }
            } else if (dCm > 8.0) armed = true;

            telemetry.addData("Dist (m)", "%.2f", currentDistMeters);
            telemetry.addData("Mode", (currentDistMeters > THRESHOLD_METERS) ? "FAR" : "CLOSE");
            telemetry.addData("RPM", "%.0f (Trim: %d)", shooter1.getVelocity(), rpmOffset);
            telemetry.addData("Hood", "%.2f", hood.getPosition());
            telemetry.update();
        }
    }

    // =========================================================
    // SIMPLIFIED PHYSICS ENGINE
    // =========================================================
    private void solvePhysicsShot(double distMeters) {

        double targetAngleDeg;

        if (distMeters > THRESHOLD_METERS) {
            hood.setPosition(HOOD_FAR_POS);
            targetAngleDeg = HOOD_FAR_DEG;
        } else {
            hood.setPosition(HOOD_CLOSE_POS);
            targetAngleDeg = HOOD_CLOSE_DEG;
        }

        double thetaRad = Math.toRadians(targetAngleDeg);
        double g = GRAVITY;
        double x = distMeters;
        double y = DELTA_H_M;

        double num = g * x * x;
        double denTerm1 = 2 * Math.pow(Math.cos(thetaRad), 2);
        double denTerm2 = (x * Math.tan(thetaRad)) - y;

        if (denTerm2 <= 0) denTerm2 = 0.1;

        double velocityMPS = Math.sqrt(num / (denTerm1 * denTerm2));

        double circumferenceM = (WHEEL_DIAMETER_CM / 100.0) * Math.PI;
        double autoRPM = (velocityMPS / circumferenceM) * 60.0;

        double finalRPM = autoRPM + rpmOffset;
        if(finalRPM < 0) finalRPM = 1000;
        if(finalRPM > 3200) finalRPM = 3200;

        shooter1.setVelocity(finalRPM);
        shooter2.setVelocity(finalRPM);
    }

    // =========================================================
    // MECHANISMS
    // =========================================================
    private void updateMechanisms(long now) {
        if(indexerBusy && now > indexerBusyUntilMs) indexerBusy = false;

        if(resetState == ResetState.KICK_DOWN && now - resetT0 > 250) {
            goToSlot(nextFillSlot); resetState = ResetState.INDEX_TO_NEXT;
        } else if (resetState == ResetState.INDEX_TO_NEXT && !indexerBusy) resetState = ResetState.NONE;

        switch(shootState) {
            case INDEX_TO_SHOOT:
                if(!indexerBusy) { kicker.setPosition(KICKER_UP); shootState=ShootState.KICK_UP; shootT0=now; } break;
            case KICK_UP:
                if(now-shootT0 > 350) { shootState=ShootState.HOLD; shootT0=now; } break;
            case HOLD:
                if(now-shootT0 > 140) { kicker.setPosition(KICKER_DOWN); shootState=ShootState.KICK_DOWN; shootT0=now; } break;
            case KICK_DOWN:
                if(now-shootT0 > 450) {
                    if(clearSlotMemoryOnFinish && shootingSlot!=-1) {
                        slotFull[shootingSlot]=false; slotColor[shootingSlot]=BallColor.EMPTY;
                    }
                    shootState=ShootState.IDLE;
                } break;
        }
    }

    private void startShoot(int slot, boolean mem) {
        shootingSlot=slot; clearSlotMemoryOnFinish=mem;
        shootState=ShootState.INDEX_TO_SHOOT; shootT0=System.currentTimeMillis();
        intakeEnabled=false;
        setIndexer(SHOOT_POS[slot]);
    }

    private void storeBall(BallColor c) {
        recalcNextFillSlot(); if(nextFillSlot<0)return;
        slotColor[nextFillSlot]=c; slotFull[nextFillSlot]=true;
        recalcNextFillSlot(); if(nextFillSlot>=0) goToSlot(nextFillSlot);
    }

    private void startReset() {
        clearAll(); recalcNextFillSlot(); intakeEnabled=true; armed=true;
        kicker.setPosition(KICKER_DOWN); resetState=ResetState.KICK_DOWN; resetT0=System.currentTimeMillis();
    }

    private void setIndexer(double p) {
        indexer.setPosition(p); indexerBusy=true; indexerBusyUntilMs=System.currentTimeMillis()+300;
    }
    private void goToSlot(int i) { if(i>=0) setIndexer(SLOT_POS[i]); }
    private void recalcNextFillSlot() { nextFillSlot=-1; for(int i=0;i<3;i++) if(!slotFull[i]){nextFillSlot=i;break;} }
    private boolean isFull() { return slotFull[0]&&slotFull[1]&&slotFull[2]; }
    private void clearAll() { for(int i=0;i<3;i++) { slotFull[i]=false; slotColor[i]=BallColor.EMPTY; } }
    private int findSlot(BallColor c) { for(int i=2;i>=0;i--) if(slotFull[i] && slotColor[i]==c) return i; return -1; }
}