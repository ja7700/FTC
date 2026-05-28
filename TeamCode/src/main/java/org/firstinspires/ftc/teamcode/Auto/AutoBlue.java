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
 * BLUE alliance autonomous for the DECODE shooter robot.
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
 *
 * =========================================================================
 *  COORDINATE FRAME  -- standard Road Runner / FTC convention
 * =========================================================================
 *  Origin: field center.  +X toward the back wall you start nearest.
 *  +Y to the LEFT.  Heading 0 faces +X, CCW positive.  Units: INCHES
 *  (Road Runner works in inches -- note your TELEOP used mm; keep them separate).
 *  Field is 12 ft = 144 in, so each wall sits at +/- 72 in.
 *
 *  All poses below are STARTING ESTIMATES placed to match the field diagram with
 *  the robot starting in the RED area (upper-right of the manual figure). They are
 *  marked TODO where you must verify against your real robot/shot. The numbers are
 *  internally consistent and safe (nothing crosses the midline), but you WILL want
 *  to nudge SHOOT_POSE until your flywheels actually make the shot, and verify the
 *  intake poses against where the artifact sets sit on your field.
 * =========================================================================
 */
@Autonomous(name = "Auto BLUE", group = "Auto")
public class AutoBlue extends LinearOpMode {

    // ====================== ALLIANCE ======================
    // This file is BLUE. AutoRed.java is identical with IS_RED=true (it mirrors Y).
    private static final boolean IS_RED = false;

    // ====================== POSES (inches) ======================
    // Helper: mirror Y for blue. For RED this is identity.
    private static double my(double y) { return IS_RED ? y : -y; }
    private static double mh(double headingRad) { return IS_RED ? headingRad : -headingRad; }

    // --- Start pose: against the back wall in the RED alliance area (upper-right of figure).
    // x near the +X wall (72 in), y on the red (left-of-figure-from-driver) side, facing the field (-X)
    // TODO: measure your exact start. Robot back against the wall tile edge.
    private static final Pose2d START_POSE =
            new Pose2d(48.0, my(48.0), Math.toRadians(my(180.0)));

    // --- Shooting spot: backed up toward the goal but NOT across the midline (x stays > 0).
    // TODO: tune this until the shot actually goes in. This is the single most important pose.
    private static final Pose2d SHOOT_POSE =
            new Pose2d(24.0, my(48.0), Math.toRadians(my(135.0)));

    // --- Artifact set 1 (closest row to start). Heading points the intake at the artifacts.
    // TODO: verify against the artifact set positions in the manual figure for your alliance.
    private static final Pose2d INTAKE1_POSE =
            new Pose2d(24.0, my(24.0), Math.toRadians(my(-135.0)));

    // --- Artifact set 2 (next row). TODO verify.
    private static final Pose2d INTAKE2_POSE =
            new Pose2d(24.0, my(0.0), Math.toRadians(my(-135.0)));

    // --- Park: stay on your alliance side, out of the way. TODO confirm legal park zone.
    private static final Pose2d PARK_POSE =
            new Pose2d(48.0, my(36.0), Math.toRadians(my(180.0)));

    // ====================== HARDWARE ======================
    private DcMotor intake, shooter;
    private DcMotorEx flywheel1, flywheel2;
    private Servo hood, gate;

    // ====================== CONSTANTS (mirrored from Teleop.java) ======================
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM    = 5500.0;
    private static final double TARGET_TPS    = (TARGET_RPM / 60.0) * TICKS_PER_REV;
    private static final double READY_RPM     = 5225.0;
    private static final double READY_TPS     = (READY_RPM / 60.0) * TICKS_PER_REV;

    private static final double INTAKE_POWER  = 1.0;
    private static final double SHOOTER_POWER = 1.0;

    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    private static final double GATE_OPEN_DEG   = 11.0;
    private static final double GATE_CLOSED_DEG = 45.0;

    private static final double HOOD_MIN_DEG = 0.0;
    // Fixed hood angle for the auto shooting spot. Distance-based logic from teleop is
    // overkill here since we shoot from one spot; just pick the angle that works there.
    // TODO: set to whatever hood angle makes the SHOOT_POSE shot.
    private static final double HOOD_SHOOT_DEG = 60.0;

    // ====================== TIMINGS (seconds) ======================
    private static final double SPINUP_TIME   = 1.5;  // initial flywheel spin-up
    private static final double SHOOT_TIME    = 1.5;  // time to feed all preloads through
    private static final double SETTLE_TIME   = 0.3;  // pause after gate moves before shooting/intaking
    private static final double INTAKE_TIME   = 1.5;  // time to suck in a set while driving onto it

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

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        flywheel1.setVelocity(TARGET_TPS);
        flywheel2.setVelocity(TARGET_TPS);

        // ============================================================
        //  BUILD THE FULL AUTONOMOUS AS ONE RR ACTION
        // ============================================================
        Action auto = new SequentialAction(
                // ---- 1) Drive to shooting spot while flywheels spin up ----
                drive.actionBuilder(START_POSE)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                waitForFlywheel(),               // make sure we're actually at speed
                new SleepAction(SETTLE_TIME),

                // ---- 2) Shoot preloads (gate OPEN) ----
                openGate(),
                new SleepAction(SETTLE_TIME),
                shoot(SHOOT_TIME),
                closeGate(),                     // close before driving to intake

                // ---- 3) Go to set 1, intake (gate CLOSED) ----
                drive.actionBuilder(SHOOT_POSE)
                        .strafeToLinearHeading(INTAKE1_POSE.position, INTAKE1_POSE.heading)
                        .build(),
                intakeSet(INTAKE_TIME),

                // ---- 4) Return to shoot, shoot ----
                drive.actionBuilder(INTAKE1_POSE)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                waitForFlywheel(),
                openGate(),
                new SleepAction(SETTLE_TIME),
                shoot(SHOOT_TIME),
                closeGate(),

                // ---- 5) Go to set 2, intake ----
                drive.actionBuilder(SHOOT_POSE)
                        .strafeToLinearHeading(INTAKE2_POSE.position, INTAKE2_POSE.heading)
                        .build(),
                intakeSet(INTAKE_TIME),

                // ---- 6) Return to shoot, shoot ----
                drive.actionBuilder(INTAKE2_POSE)
                        .strafeToLinearHeading(SHOOT_POSE.position, SHOOT_POSE.heading)
                        .build(),
                waitForFlywheel(),
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
        return new TimedAction(seconds,
                () -> {
                    flywheel1.setVelocity(TARGET_TPS);
                    flywheel2.setVelocity(TARGET_TPS);
                    shooter.setPower(SHOOTER_POWER);
                    // optional: a touch of intake to keep balls fed up to the shooter
                    intake.setPower(INTAKE_POWER);
                },
                () -> {
                    shooter.setPower(0.0);
                    intake.setPower(0.0);
                });
    }

    /** Run intake in for `seconds` (gate must already be CLOSED). */
    private Action intakeSet(double seconds) {
        return new TimedAction(seconds,
                () -> intake.setPower(INTAKE_POWER),
                () -> intake.setPower(0.0));
    }

    /** Block until flywheels reach the ready threshold (with a timeout safety). */
    private Action waitForFlywheel() {
        return new Action() {
            double t0 = -1;
            @Override public boolean run(@NonNull TelemetryPacket p) {
                if (t0 < 0) t0 = now();
                double avgTps = (Math.abs(flywheel1.getVelocity()) + Math.abs(flywheel2.getVelocity())) / 2.0;
                boolean ready = avgTps >= READY_TPS;
                boolean timedOut = (now() - t0) > 3.0; // never hang forever
                return !(ready || timedOut); // keep running while NOT ready and NOT timed out
            }
        };
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
        flywheel1.setVelocity(0);
        flywheel2.setVelocity(0);
    }

    private static String poseStr(Pose2d p) {
        return String.format("(%.1f, %.1f) @ %.0f deg",
                p.position.x, p.position.y, Math.toDegrees(p.heading.toDouble()));
    }
}