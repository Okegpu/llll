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
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
@TeleOp(name="Drive + Indexer + Turret + Shooter Modes", group="Drive")
public class Micks extends LinearOpMode {

    private DcMotor fLMotor, fRMotor, bLMotor, bRMotor;

    private static final double DB = 0.06;
    private static final double STRAFE_SCALE = 1.10;

    private static final boolean REVERSE_LEFT_SIDE  = true;
    private static final boolean REVERSE_RIGHT_SIDE = false;

    // =======================
    // INTAKE/OUTTAKE MOTOR (evil)
    // =======================
    private DcMotor evil;
    private boolean evilOn = false;
    private boolean lastLB = false;

    // Hold LT to outtake (reverse). Outtake overrides intake toggle.
    private static final double INTAKE_POWER  =  1.0;
    private static final double OUTTAKE_POWER = -1.0;
    private static final double OUTTAKE_TRIGGER_THRESH = 0.60;

    // =======================
    // INDEXER + KICKER + SENSORS
    // =======================
    private Servo indexer, kicker;
    private ColorSensor color;
    private DistanceSensor distance;

    // =======================
    // LIMELIGHT
    // =======================
    private Limelight3A limelight;
    private static final String LIMELIGHT_NAME = "limelight";
    private static final int APRILTAG_PIPELINE = 0;

    private boolean llStarted = false;
    private long lastValidTagMs = 0;

    // =======================
    // TURRET (MOTOR)
    // =======================
    private DcMotorEx turret;

    private static final double MOTOR_OUT_TICKS_PER_REV = 383.6;
    private static final double EXTRA_GEAR_RATIO = 100.0 / 54.0;
    private static final double TICKS_PER_TURRET_REV = MOTOR_OUT_TICKS_PER_REV * EXTRA_GEAR_RATIO;

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG = +90.0;
    private static final int TURRET_ZERO_OFFSET_TICKS = 0;

    // TX tracking
    private static final double TX_kP = 0.020;
    private static final double TX_kD = 0.001;
    private static final double TX_DEADBAND_DEG = 1.0;
    private static final double TX_MAX_POWER = 0.9;

    private double lastTx = 0.0;
    private long lastTxMs = 0;

    private static final boolean INVERT_TX_POWER = true;

    // =======================
    // SHOOTER (TWO MOTORS, ALWAYS ON)
    // =======================
    private DcMotorEx shooter1, shooter2;

    private static final String SHOOTER1_NAME = "shooter1";
    private static final String SHOOTER2_NAME = "shooter2";

    private static final boolean REVERSE_SHOOTER_1 = false;
    private static final boolean REVERSE_SHOOTER_2 = true;

    // Velocity modes (RUN_USING_ENCODER)
    private static final double VEL_SOFT   = 700;
    private static final double VEL_MEDIUM = 1500;
    private static final double VEL_STRONG = 2600;

    private enum ShotMode { SOFT, MEDIUM, STRONG }
    private ShotMode shotMode = ShotMode.MEDIUM;

    // DPAD tracking (+ RIGHT for manual cycle shoot)
    private boolean lastDpadUp=false, lastDpadLeft=false, lastDpadDown=false, lastDpadRight=false;

    // =======================
    // MANUAL SHOOT (DPAD RIGHT) — cycles 1->2->3 regardless of detection/memory
    // =======================
    private int manualShootSlot = 0; // 0,1,2

    // =======================
    // INDEXER POSITIONS
    // =======================
    private final double[] SLOT_POS  = {0.20, 0.38, 0.59};
    private final double[] SHOOT_POS = {0.50, 0.68, 0.89};

    // Kicker
    private static final double KICKER_DOWN = 1.00;
    private static final double KICKER_UP   = 0.35;

    private static final boolean KICKER_REVERSED = false;
    private double kickerDown() { return KICKER_REVERSED ? (1.0 - KICKER_DOWN) : KICKER_DOWN; }
    private double kickerUp()   { return KICKER_REVERSED ? (1.0 - KICKER_UP)   : KICKER_UP;   }

    // =======================
    // TIMING (NO SLEEPS IN LOOP)
    // =======================
    private static final long INDEXER_SETTLE_MS     = 180;

    private static final long KICKER_UP_TRAVEL_MS   = 350;
    private static final long KICKER_PEAK_HOLD_MS   = 140;
    private static final long KICKER_DOWN_TRAVEL_MS = 450;

    private static final long EXTRA_HOLD_AFTER_INDEXER_MS = 120;
    private static final long KICKER_MAX_UP_MS = 900;

    private static final long MOVE_LOCKOUT_MS = 250;
    private long ignoreDetectUntilMs = 0;

    // =======================
    // HSV + DISTANCE DETECTION
    // =======================
    private static final double MAX_DIST_CM = 6.0;
    private static final double LEAVE_DIST_CM = 8.0;

    private static final int SAMPLES = 5;

    private static final float MIN_SAT = 0.25f;
    private static final float MIN_VAL = 0.15f;

    private static final float GREEN_HUE_MIN = 80f;
    private static final float GREEN_HUE_MAX = 165f;

    private static final float PURPLE_HUE_MIN = 200f;
    private static final float PURPLE_HUE_MAX = 345f;

    private static final int DET_NEED = 3;
    private BallColor lastDet = BallColor.EMPTY;
    private int detCount = 0;

    private static final long STORE_COOLDOWN_MS = 350;
    private long lastStoreMs = 0;

    public enum BallColor { EMPTY, GREEN, PURPLE }

    private final BallColor[] slotColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private final boolean[] slotFull = {false,false,false};

    private int nextFillSlot = 0;
    private boolean armed = true;
    private boolean intakeEnabled = true;

    private boolean lastB=false,lastY=false,lastA=false,lastX=false;

    // =======================
    // NON-BLOCKING SHOOT STATE (INDEXER/KICKER)
    // =======================
    private enum ShootState { IDLE, INDEX_TO_SHOOT, KICK_UP, HOLD, KICK_DOWN }
    private ShootState shootState = ShootState.IDLE;
    private long shootT0 = 0;
    private int shootingSlot = -1;

    // If false => manual/blind shot: do NOT clear slot memory when finished
    private boolean clearSlotMemoryOnFinish = true;

    // =======================
    // NON-BLOCKING INDEXER MOVE
    // =======================
    private boolean indexerBusy = false;
    private long indexerBusyUntilMs = 0;
    private double lastIndexerPos = 0.20;

    // =======================
    // NON-BLOCKING RESET
    // =======================
    private enum ResetState { NONE, KICK_DOWN, INDEX_TO_NEXT }
    private ResetState resetState = ResetState.NONE;
    private long resetT0 = 0;

    @Override
    public void runOpMode() {

        // DRIVE
        fLMotor = hardwareMap.get(DcMotor.class,"front_left");
        fRMotor = hardwareMap.get(DcMotor.class,"front_right");
        bLMotor = hardwareMap.get(DcMotor.class,"back_left");
        bRMotor = hardwareMap.get(DcMotor.class,"back_right");

        if (REVERSE_LEFT_SIDE) {
            fLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            bLMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (REVERSE_RIGHT_SIDE) {
            fRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            bRMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        fLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INTAKE/OUTTAKE (evil)
        evil = hardwareMap.get(DcMotor.class,"evil");
        evil.setPower(0);
        evil.setDirection(DcMotorSimple.Direction.REVERSE);

        // INDEXER + KICKER
        indexer = hardwareMap.get(Servo.class,"indexer");
        kicker  = hardwareMap.get(Servo.class,"kick");

        // TURRET
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);

        // SHOOTERS
        shooter1 = hardwareMap.get(DcMotorEx.class, SHOOTER1_NAME);
        shooter2 = hardwareMap.get(DcMotorEx.class, SHOOTER2_NAME);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(REVERSE_SHOOTER_1 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(REVERSE_SHOOTER_2 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // LIMELIGHT
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        try { limelight.setPollRateHz(100); } catch (Exception ignored) {}
        try {
            limelight.start();
            llStarted = true;
        } catch (Exception e) {
            llStarted = false;
        }

        // SENSORS
        color = hardwareMap.get(ColorSensor.class,"color");
        distance = hardwareMap.get(DistanceSensor.class,"color");
        color.enableLed(true);

        // INIT positions (sleeps OK pre-start)
        kicker.setPosition(kickerDown());
        sleep(250);

        clearAll();
        recalcNextFillSlot();
        goToSlot(nextFillSlot);
        sleep(200);

        // Start shooters immediately
        applyShotMode(shotMode);

        telemetry.addLine("READY");
        telemetry.addData("Limelight started", llStarted);
        telemetry.addData("AprilTag pipeline", APRILTAG_PIPELINE);
        telemetry.addData("Ticks/TurretRev", "%.2f", TICKS_PER_TURRET_REV);
        telemetry.addData("ShotMode", shotMode);
        telemetry.addLine("Dpad: UP=Strong  LEFT=Medium  DOWN=Soft  RIGHT=MANUAL SHOOT 1-2-3");
        telemetry.addLine("LB=Intake toggle   LT(hold)=Outtake");
        telemetry.addLine("B=Shoot Green (memory)   Y=Shoot Purple (memory)   X=Reset memory   A=Return intake");
        telemetry.update();

        waitForStart();

        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        applyShotMode(shotMode);

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // =======================
            // DRIVE
            // =======================
            double x  =  gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x;

            if(Math.abs(x)<DB)x=0;
            if(Math.abs(y)<DB)y=0;
            if(Math.abs(rx)<DB)rx=0;

            mecanumDrive(x, y, rx);

            // =======================
            // INTAKE / OUTTAKE
            // =======================
            boolean outtakeHeld = (gamepad1.left_trigger > OUTTAKE_TRIGGER_THRESH);

            boolean lb = gamepad1.left_bumper;
            if(lb && !lastLB){
                evilOn = !evilOn;
            }
            lastLB = lb;

            if (outtakeHeld) {
                evil.setPower(OUTTAKE_POWER);
                // keep sensors from trying to auto-store while spitting
                intakeEnabled = false;
                armed = false;
                resetDetConfidence();
            } else {
                evil.setPower(evilOn ? INTAKE_POWER : 0.0);
                if (evilOn) intakeEnabled = true;
            }

            // =======================
            // SHOOT MODE SWITCH (DPAD UP/LEFT/DOWN)
            // =======================
            boolean dUp = gamepad1.dpad_up;
            boolean dLeft = gamepad1.dpad_left;
            boolean dDown = gamepad1.dpad_down;
            boolean dRight = gamepad1.dpad_right;

            // manual shoot edge (DPAD RIGHT)
            boolean manualShootPressed = dRight && !lastDpadRight;

            if(dUp && !lastDpadUp){
                shotMode = ShotMode.STRONG;
            }
            if(dLeft && !lastDpadLeft){
                shotMode = ShotMode.MEDIUM;
            }
            if(dDown && !lastDpadDown){
                shotMode = ShotMode.SOFT;
            }

            lastDpadUp = dUp;
            lastDpadLeft = dLeft;
            lastDpadDown = dDown;
            lastDpadRight = dRight;

            // Shooters always commanded
            applyShotMode(shotMode);

            // =======================
            // TURRET TRACK (always)
            // =======================
            TurretLLData ll = readLimelight();
            updateTurretTracking(now, ll);

            // =======================
            // UPDATE NON-BLOCKING STATES
            // =======================
            updateIndexerBusy(now);
            updateShoot(now);
            updateReset(now);

            boolean servoBusy = (now < ignoreDetectUntilMs) || indexerBusy ||
                    (shootState != ShootState.IDLE) || (resetState != ResetState.NONE);

            // =======================
            // INPUT EDGES
            // =======================
            boolean bPressed=gamepad1.b&&!lastB;
            boolean yPressed=gamepad1.y&&!lastY;
            boolean aPressed=gamepad1.a&&!lastA;
            boolean xPressed=gamepad1.x&&!lastX;

            lastB=gamepad1.b;
            lastY=gamepad1.y;
            lastA=gamepad1.a;
            lastX=gamepad1.x;

            // RESET MEMORY (X)
            if(xPressed && resetState == ResetState.NONE){
                startReset();
            }

            // A = return to intake mode
            if(aPressed && !servoBusy){
                intakeEnabled=true;
                armed=true;
                recalcNextFillSlot();
                kicker.setPosition(kickerDown());
                goToSlot(nextFillSlot);
            }

            // =======================
            // MANUAL SHOOT (DPAD RIGHT): cycles 1-2-3 no matter what
            // =======================
            if(manualShootPressed && !servoBusy && shootState == ShootState.IDLE){
                startShoot(manualShootSlot, false); // false = do NOT clear memory
                manualShootSlot++;
                if(manualShootSlot > 2) manualShootSlot = 0;
            }

            // =======================
            // DISTANCE / ARMING
            // =======================
            double distCm = distance.getDistance(DistanceUnit.CM);
            boolean distOk = Double.isFinite(distCm);

            if(intakeEnabled && !armed && distOk && distCm > LEAVE_DIST_CM){
                armed = true;
                resetDetConfidence();
            }

            // =======================
            // SMART SHOOT REQUESTS (B=Green, Y=Purple) uses memory
            // =======================
            if(!servoBusy && shootState == ShootState.IDLE){
                if(bPressed) {
                    int i = findSlotWithColorClosestToShooter(BallColor.GREEN);
                    if(i != -1) startShoot(i, true); // true = clear memory for that slot
                }
                if(yPressed) {
                    int i = findSlotWithColorClosestToShooter(BallColor.PURPLE);
                    if(i != -1) startShoot(i, true);
                }
            }

            // =======================
            // AUTO STORE
            // =======================
            if(intakeEnabled && !servoBusy && armed && !isFull() && distOk && distCm <= MAX_DIST_CM){
                BallColor detected = detectBallHSV().detected;

                if(detected==BallColor.GREEN||detected==BallColor.PURPLE){
                    if(detected==lastDet) detCount++;
                    else { lastDet=detected; detCount=1; }

                    if(detCount>=DET_NEED && now-lastStoreMs>STORE_COOLDOWN_MS){
                        storeBallAndAdvance(detected);
                        lastStoreMs=now;
                        armed=false;
                        resetDetConfidence();
                    }
                } else {
                    resetDetConfidence();
                }
            }

            // =======================
            // TELEMETRY
            // =======================
            telemetry.addData("ShotMode", shotMode);
            telemetry.addData("ManualNextSlot", (manualShootSlot+1)); // 1..3
            telemetry.addData("IntakeToggle", evilOn);
            telemetry.addData("OuttakeHeld", outtakeHeld);

            telemetry.addData("TargetVel", "%.0f", getTargetVelocity(shotMode));
            telemetry.addData("S1 vel", "%.0f", shooter1.getVelocity());
            telemetry.addData("S2 vel", "%.0f", shooter2.getVelocity());

            telemetry.addData("TurretDeg", "%.1f", getTurretDeg());
            telemetry.addData("TagSeen", ll.tagSeen);
            telemetry.addData("TX(deg)", ll.tagSeen ? String.format("%.2f", ll.txDeg) : "--");
            telemetry.addData("LastTag(ms)", (lastValidTagMs==0) ? "--" : String.valueOf(now-lastValidTagMs));

            telemetry.addData("Dist(cm)", distOk ? String.format("%.1f", distCm) : "NaN");
            telemetry.addData("Slots(mem)", "%s | %s | %s", slotColor[0],slotColor[1],slotColor[2]);
            telemetry.update();
        }
    }

    // =======================
    // SHOOTER HELPERS
    // =======================
    private double getTargetVelocity(ShotMode m){
        switch(m){
            case SOFT:   return VEL_SOFT;
            case MEDIUM: return VEL_MEDIUM;
            case STRONG: return VEL_STRONG;
        }
        return VEL_MEDIUM;
    }

    private void applyShotMode(ShotMode m){
        double v = getTargetVelocity(m);
        shooter1.setVelocity(v);
        shooter2.setVelocity(v);
    }

    // =========================================================
    // LIMELIGHT READ + TURRET TRACKING
    // =========================================================
    private static class TurretLLData {
        boolean tagSeen = false;
        double txDeg = Double.NaN;
    }

    private TurretLLData readLimelight(){
        TurretLLData out = new TurretLLData();

        LLResult result = limelight.getLatestResult();
        if(result == null) return out;

        boolean valid;
        try { valid = result.isValid(); }
        catch (Exception e) { valid = false; }

        if(!valid) return out;

        try {
            if(result.getFiducialResults() != null && result.getFiducialResults().size() <= 0){
                return out;
            }
        } catch (Exception ignored) {}

        try {
            out.txDeg = result.getTx();
            out.tagSeen = Double.isFinite(out.txDeg);
        } catch (Exception ignored) {
            out.tagSeen = false;
        }

        return out;
    }

    private void updateTurretTracking(long now, TurretLLData ll){
        if (ll.tagSeen) {
            lastValidTagMs = now;

            double txDeg = ll.txDeg;
            double power;

            if (Math.abs(txDeg) <= TX_DEADBAND_DEG) {
                power = 0.0;
            } else {
                double dt = Math.max(0.001, (now - lastTxMs) / 1000.0);
                double dtx = (txDeg - lastTx) / dt;

                power = (TX_kP * txDeg) + (TX_kD * dtx);
                if (INVERT_TX_POWER) power = -power;

                power = clamp(power, -TX_MAX_POWER, TX_MAX_POWER);
            }

            double turretDeg = getTurretDeg();
            if (turretDeg <= TURRET_MIN_DEG + 1.0 && power < 0) power = 0;
            if (turretDeg >= TURRET_MAX_DEG - 1.0 && power > 0) power = 0;

            turret.setPower(power);

            lastTx = txDeg;
            lastTxMs = now;
        } else {
            turret.setPower(0);
        }
    }

    private double getTurretDeg(){
        int ticks = turret.getCurrentPosition() - TURRET_ZERO_OFFSET_TICKS;
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    // =========================================================
    // HSV DETECTION
    // =========================================================
    private static class DetResult{
        BallColor detected=BallColor.EMPTY;
    }

    private DetResult detectBallHSV(){
        DetResult out=new DetResult();

        double r=0,g=0,b=0;
        for(int i=0;i<SAMPLES;i++){
            r+=color.red();
            g+=color.green();
            b+=color.blue();
        }
        r/=SAMPLES;g/=SAMPLES;b/=SAMPLES;

        double sum=r+g+b;
        if(sum<1)return out;

        int R=(int)(255*(r/sum));
        int G=(int)(255*(g/sum));
        int B=(int)(255*(b/sum));

        float[] hsv=new float[3];
        Color.RGBToHSV(R,G,B,hsv);

        float h=hsv[0],s=hsv[1],v=hsv[2];
        if(s<MIN_SAT||v<MIN_VAL)return out;

        if(h>=GREEN_HUE_MIN&&h<=GREEN_HUE_MAX) out.detected=BallColor.GREEN;
        else if(h>=PURPLE_HUE_MIN&&h<=PURPLE_HUE_MAX) out.detected=BallColor.PURPLE;

        return out;
    }

    private void resetDetConfidence(){detCount=0;lastDet=BallColor.EMPTY;}

    // =========================================================
    // STORE / SHOOT (INDEXER/KICKER)
    // =========================================================
    private void storeBallAndAdvance(BallColor c){
        recalcNextFillSlot();
        if(nextFillSlot<0)return;

        slotColor[nextFillSlot]=c;
        slotFull[nextFillSlot]=true;

        recalcNextFillSlot();
        if(nextFillSlot>=0){
            goToSlot(nextFillSlot);
        }
    }

    private void startShoot(int slot){
        startShoot(slot, true);
    }

    // clearMemoryOnFinish=false => manual/blind shoot
    private void startShoot(int slot, boolean clearMemoryOnFinish){
        shootingSlot = slot;
        clearSlotMemoryOnFinish = clearMemoryOnFinish;

        shootState = ShootState.INDEX_TO_SHOOT;
        shootT0 = System.currentTimeMillis();

        intakeEnabled = false;

        setIndexerPosition(SHOOT_POS[slot]);
        ignoreDetectUntilMs = shootT0 + MOVE_LOCKOUT_MS;
    }

    private void updateShoot(long now){
        switch(shootState){
            case IDLE:
                return;

            case INDEX_TO_SHOOT:
                if(!indexerBusy){
                    kicker.setPosition(kickerUp());
                    shootState = ShootState.KICK_UP;
                    shootT0 = now;
                }
                break;

            case KICK_UP:
                if(now - shootT0 >= KICKER_UP_TRAVEL_MS){
                    shootState = ShootState.HOLD;
                    shootT0 = now;
                }
                break;

            case HOLD: {
                boolean minHoldDone = (now - shootT0) >= KICKER_PEAK_HOLD_MS;
                boolean indexerDone = !indexerBusy;
                boolean extraSettleDone = (now >= indexerBusyUntilMs + EXTRA_HOLD_AFTER_INDEXER_MS);
                boolean maxUpExceeded = (now - shootT0) >= KICKER_MAX_UP_MS;

                if( (minHoldDone && indexerDone && extraSettleDone) || maxUpExceeded ){
                    kicker.setPosition(kickerDown());
                    shootState = ShootState.KICK_DOWN;
                    shootT0 = now;
                }
                break;
            }

            case KICK_DOWN:
                if(now - shootT0 >= KICKER_DOWN_TRAVEL_MS){
                    // Only clear memory if this shot was a "smart" memory-based shot
                    if(clearSlotMemoryOnFinish && shootingSlot >= 0 && shootingSlot < 3){
                        slotFull[shootingSlot]=false;
                        slotColor[shootingSlot]=BallColor.EMPTY;
                    }
                    shootingSlot = -1;
                    shootState = ShootState.IDLE;
                }
                break;
        }
    }

    private int findSlotWithColorClosestToShooter(BallColor t){
        for(int i=2;i>=0;i--)
            if(slotFull[i]&&slotColor[i]==t)return i;
        return -1;
    }

    private void recalcNextFillSlot(){
        nextFillSlot=-1;
        for(int i=0;i<3;i++)
            if(!slotFull[i]){nextFillSlot=i;break;}
    }

    private void setIndexerPosition(double pos){
        indexer.setPosition(pos);

        double delta = Math.abs(pos - lastIndexerPos);
        lastIndexerPos = pos;

        long travelMs = (long)(90 + (delta * 500)); // tune if needed
        long settleMs = INDEXER_SETTLE_MS;

        markIndexerBusy(travelMs + settleMs);
        ignoreDetectUntilMs = System.currentTimeMillis() + MOVE_LOCKOUT_MS;
    }

    private void goToSlot(int i){
        if(i<0)return;
        setIndexerPosition(SLOT_POS[i]);
    }

    private boolean isFull(){
        return slotFull[0]&&slotFull[1]&&slotFull[2];
    }

    private void clearAll(){
        for(int i=0;i<3;i++){
            slotFull[i]=false;
            slotColor[i]=BallColor.EMPTY;
        }
    }

    // =========================================================
    // RESET (NON-BLOCKING)
    // =========================================================
    private void startReset(){
        clearAll();
        recalcNextFillSlot();
        intakeEnabled = true;
        armed = true;
        resetDetConfidence();

        // restart manual order at slot1
        manualShootSlot = 0;

        kicker.setPosition(kickerDown());
        resetState = ResetState.KICK_DOWN;
        resetT0 = System.currentTimeMillis();
    }

    private void updateReset(long now){
        switch(resetState){
            case NONE: return;

            case KICK_DOWN:
                if(now - resetT0 >= 250){
                    goToSlot(nextFillSlot);
                    resetState = ResetState.INDEX_TO_NEXT;
                    resetT0 = now;
                }
                break;

            case INDEX_TO_NEXT:
                if(!indexerBusy){
                    resetState = ResetState.NONE;
                }
                break;
        }
    }

    // =========================================================
    // INDEXER BUSY HELPERS
    // =========================================================
    private void markIndexerBusy(long durationMs){
        long now = System.currentTimeMillis();
        indexerBusy = true;
        indexerBusyUntilMs = Math.max(indexerBusyUntilMs, now + durationMs);
    }

    private void updateIndexerBusy(long now){
        if(indexerBusy && now >= indexerBusyUntilMs){
            indexerBusy = false;
        }
    }

    // =========================================================
    // DRIVE
    // =========================================================
    private void mecanumDrive(double x, double y, double rx){

        x *= STRAFE_SCALE;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max; fr /= max; bl /= max; br /= max;

        fLMotor.setPower(fl);
        fRMotor.setPower(fr);
        bLMotor.setPower(bl);
        bRMotor.setPower(br);
    }

    // =========================================================
    // MATH HELPERS
    // =========================================================
    private static double clamp(double v, double lo, double hi){
        return Math.max(lo, Math.min(hi, v));
    }
}
