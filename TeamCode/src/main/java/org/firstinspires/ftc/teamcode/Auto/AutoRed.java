package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Road Runner 1.0 MecanumDrive. Your MecanumDrive is wired to localize via the goBILDA
// Pinpoint through PinpointLocalizer, so we just use MecanumDrive directly here.
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * RED alliance autonomous for the DECODE shooter robot.
 *
 * SEQUENCE (matches the teleop hardware in Teleop.java):
 *   1. Close gate, spin up flywheels.
 *   2. Drive back off the wall toward the goal-side shooting spot (does NOT cross midline).
 *   3. Open gate, shoot preloads, close gate.
 *   4. Turn toward artifact set 1, intake it (gate stays CLOSED while intaking).
 *   5. Drive back to the shooting spot, open gate, shoot, close gate.
 *   6. Intake set 2 (gate CLOSED), return, shoot.
 *   ( your message said "3rd set" -> set 1 = preload-adjacent, sets 2 & 3 are the two
 *     on-field rows we drive out to. 3 shooting volleys total. )
 *   7. Park.
 *
 * GATE RULE (from your spec): gate must be CLOSED before intaking and OPEN before shooting.
 */
@Autonomous(name = "Auto RED", group = "Auto")
public class AutoRed extends LinearOpMode {

    // ====================== ALLIANCE ======================
    // This file is RED. AutoBlue.java is identical with IS_RED=false (it mirrors Y).
    private static final boolean IS_RED = true;

    // ====================== POSES (inches) ======================
    // Helper: mirror Y for blue. For RED this is identity.
    private static double my(double y) { return IS_RED ? y : -y; }
    private static double mh(double headingRad) { return IS_RED ? headingRad : -headingRad; }

    private static final Pose2d START_POSE =
            new Pose2d(55.5, my(52.5), Math.toRadians(my(45.0)));

    private static final Pose2d SHOOT_POSE =
            new Pose2d(23, my(23), Math.toRadians(my(45.0)));

    // --- Artifact row 1. Robot drives to START with intake off, switches intake on,
    // drives through to END (sweeping up all 3 balls in the row), then turns intake off.
    // Heading stays the same so the robot just translates along the row.
    // TODO: tune both poses for your real field. Currently the row runs along -X from
    // (36, 24) to (12, 24) -- adjust direction/length to match the actual artifact layout.
    private static final Pose2d INTAKE1_START =
            new Pose2d(12.0, my(12.0), Math.toRadians(my(180)));
    private static final Pose2d INTAKE1_END =
            new Pose2d(48.0, my(12.0), Math.toRadians(my(180)));

    // --- Artifact row 2. Same pattern as row 1, different Y.
    private static final Pose2d INTAKE2_START =
            new Pose2d(12.0, my(-12.0), Math.toRadians(my(180)));
    private static final Pose2d INTAKE2_END =
            new Pose2d(58.0, my(-12.0), Math.toRadians(my(180)));

    private static final Pose2d PARK_POSE =
            new Pose2d(48.0, my(30.0), Math.toRadians(my(90)));

    // ====================== HARDWARE ======================
    private DcMotor intake, shooter;
    private DcMotorEx flywheel1, flywheel2;
    private Servo hood, gate;

    // ====================== CONSTANTS (mirrored from Teleop.java) ======================
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM    = 4400;
    private static final double TARGET_TPS    = (TARGET_RPM / 60.0) * TICKS_PER_REV;
    private static final double READY_RPM     = 4350.0;
    private static final double READY_TPS     = (READY_RPM / 60.0) * TICKS_PER_REV;
    private static final double FLYWHEEL_MAX_RPM = 6000;
    private static final double INTAKE_POWER  = 1.0;
    private static final double SHOOTER_POWER = 0.5;

    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    private static final double GATE_OPEN_DEG   = 11.0;
    private static final double GATE_CLOSED_DEG = 45.0;

    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_SHOOT_DEG = 60;

    // ====================== TIMINGS (seconds) ======================
    private static final double SHOOT_TIME    = 1.5;  // time to feed all preloads through
    private static final double SETTLE_TIME   = 0.3;  // pause after gate moves before shooting/intaking

    private double rpmToPower(double rpm) {
        double p = rpm / FLYWHEEL_MAX_RPM;
        return Math.max(0.0, Math.min(1.0, p));
    }

    @Override
    public void runOpMode() {
        // ----- Hardware map (names must match your robot config) -----
        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooter   = hardwareMap.get(DcMotor.class, "shooter");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        // Directions / modes -- match teleop exactly.
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Open-loop power control (matches teleop). RUN_WITHOUT_ENCODER disables the
        // built-in velocity PID so the two flywheel motors don't fight each other on
        // a shared wheel. getVelocity() still works for telemetry/threshold reads.
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hood at a fixed shooting angle for auto.
        hood.setPosition(hoodDegToPos(HOOD_SHOOT_DEG));
        // Gate starts OPEN in teleop, but for auto we want it CLOSED before the first move
        // so artifacts don't fall out. Close it during init.
        gate.setPosition(gateDegToPos(GATE_CLOSED_DEG));

        // ----- Road Runner drive -----
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        telemetry.addLine("RED auto initialized.");
        telemetry.addData("Start", poseStr(START_POSE));
        telemetry.addData("Shoot", poseStr(SHOOT_POSE));

        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Kick the flywheels up immediately so they're ready by the time we arrive.
        double startPower = rpmToPower(TARGET_RPM);
        flywheel1.setPower(startPower);
        flywheel2.setPower(startPower);

        // ============================================================
        //  BUILD THE FULL AUTONOMOUS AS ONE RR ACTION
        // ============================================================
        Action auto = new SequentialAction(
                // ---- 1) Drive to shooting spot while flywheels spin up ----
                drive.actionBuilder(START_POSE)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                new SleepAction(SETTLE_TIME),

                // ---- 2) Shoot preloads (gate OPEN) ----
                openGate(),
                new SleepAction(SETTLE_TIME),
                shoot(SHOOT_TIME),
                closeGate(),                     // close before driving to intake

                // ---- 3) Sweep row 1: drive to START with intake off, turn intake on,
                //        drive through to END picking up all 3 balls, then turn intake off.
                drive.actionBuilder(SHOOT_POSE)
                        .strafeToLinearHeading(INTAKE1_START.position, INTAKE1_START.heading)
                        .build(),
                intakeOn(),
                drive.actionBuilder(INTAKE1_START)
                        .strafeToLinearHeading(INTAKE1_END.position, INTAKE1_END.heading)
                        .build(),
                intakeOff(),

                // ---- 4) Return to shoot, shoot ----
                drive.actionBuilder(INTAKE1_END)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                openGate(),
                new SleepAction(SETTLE_TIME),
                shoot(SHOOT_TIME),
                closeGate(),

                // ---- 5) Sweep row 2 same pattern ----
                drive.actionBuilder(SHOOT_POSE)
                        .strafeToLinearHeading(INTAKE2_START.position, INTAKE2_START.heading)
                        .build(),
                intakeOn(),
                drive.actionBuilder(INTAKE2_START)
                        .strafeToLinearHeading(INTAKE2_END.position, INTAKE2_END.heading)
                        .build(),
                intakeOff(),

                // ---- 6) Return to shoot, shoot ----
                drive.actionBuilder(INTAKE2_END)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                openGate(),
                new SleepAction(SETTLE_TIME),
                shoot(SHOOT_TIME),
                closeGate(),

                // ---- 7) Park ----
                drive.actionBuilder(SHOOT_POSE)
                        .strafeToLinearHeading(PARK_POSE.position, PARK_POSE.heading)
                        .build()
        );

        runBlocking(drive, auto);

        // Safe shutdown.
        stopAll();
    }

    // ============================================================
    //  ACTION HELPERS  (RR 1.0 Action interface: run() returns true = keep going)
    // ============================================================

    /** Spin shooter feed + keep flywheels at speed for `seconds`, then stop the feed. */
    private Action shoot(double seconds) {
        final double pwr = rpmToPower(TARGET_RPM);
        return new TimedAction(seconds,
                () -> {
                    flywheel1.setPower(pwr);
                    flywheel2.setPower(pwr);
                    shooter.setPower(SHOOTER_POWER);
                    // optional: a touch of intake to keep balls fed up to the shooter
                    intake.setPower(INTAKE_POWER);
                },
                () -> {
                    shooter.setPower(0.0);
                    intake.setPower(0.0);
                });
    }

    /** Turn intake on (gate must already be CLOSED). Used when entering an artifact row. */
    private Action intakeOn() {
        return instant(() -> intake.setPower(INTAKE_POWER));
    }

    /** Turn intake off. Used when exiting an artifact row. */
    private Action intakeOff() {
        return instant(() -> intake.setPower(0.0));
    }

    private Action openGate() {
        return instant(() -> gate.setPosition(gateDegToPos(GATE_OPEN_DEG)));
    }

    private Action closeGate() {
        return instant(() -> gate.setPosition(gateDegToPos(GATE_CLOSED_DEG)));
    }

    // ---- tiny Action utilities ----
    private interface Run { void go(); }

    private Action instant(Run r) {
        return new Action() {
            @Override public boolean run(@NonNull TelemetryPacket p) {
                r.go();
                return false; // done immediately
            }
        };
    }

    /** Runs `onTick` every loop for `seconds`, then runs `onEnd` once and finishes. */
    private class TimedAction implements Action {
        private final double seconds;
        private final Run onTick, onEnd;
        private double t0 = -1;
        TimedAction(double seconds, Run onTick, Run onEnd) {
            this.seconds = seconds; this.onTick = onTick; this.onEnd = onEnd;
        }
        @Override public boolean run(@NonNull TelemetryPacket p) {
            if (t0 < 0) t0 = now();
            if (now() - t0 >= seconds) { onEnd.go(); return false; }
            onTick.go();
            return true;
        }
    }

    private static double now() { return System.nanoTime() / 1e9; }

    // ============================================================
    //  RR ACTION RUNNER (blocking, with telemetry + early-stop)
    // ============================================================
    private void runBlocking(MecanumDrive drive, Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet =
                    new TelemetryPacket();
            running = action.run(packet);

            double avgRpm = ((Math.abs(flywheel1.getVelocity()) + Math.abs(flywheel2.getVelocity())) / 2.0
                    / TICKS_PER_REV) * 60.0;
            telemetry.addData("Avg flywheel RPM", "%.0f", avgRpm);
            telemetry.addData("Gate", "%.3f", gate.getPosition());
            telemetry.update();

            dash.sendTelemetryPacket(packet);
        }
    }

    // ============================================================
    //  SERVO MATH (mirrored from Teleop.java)
    // ============================================================
    private double hoodDegToPos(double deg) {
        double clamped = Math.max(HOOD_MIN_DEG, Math.min(SERVO_FULL_RANGE_DEG, deg));
        return clamped / SERVO_FULL_RANGE_DEG;
    }
    private double gateDegToPos(double deg) {
        double clamped = Math.max(0.0, Math.min(SERVO_FULL_RANGE_DEG, deg));
        return clamped / SERVO_FULL_RANGE_DEG;
    }

    private void stopAll() {
        intake.setPower(0);
        shooter.setPower(0);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }

    private static String poseStr(Pose2d p) {
        return String.format("(%.1f, %.1f) @ %.0f deg",
                p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}