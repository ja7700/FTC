package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

// Pinpoint odometry. GoBildaPinpointDriver is NOT part of the stock SDK -- you must add
// the file GoBildaPinpointDriver.java to your project. Get it from goBILDA's repo:
//   github.com/goBILDA-Official/FtcRobotController-Add-Pinpoint  (goBILDA-Odometry-Driver branch)
//
// IMPORTANT: the import below must point to wherever YOU placed that file. If you put it in
// the teamcode root, this is correct. If you put it in a subpackage (e.g. .Teleop or .util),
// change this import to match, or you'll get "cannot find symbol: GoBildaPinpointDriver"
// on every line that uses it.
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Intake / shooter motors
    private DcMotor intake;     // lower two wheels (one motor)
    private DcMotor shooter;    // top wheel, pushes ball into flywheels
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    // Servos
    private Servo hood;  // adjustable hood, limited to a small forward range
    private Servo gate;  // toggle gate, opens/closes on B

    // Odometry
    private GoBildaPinpointDriver pinpoint;

    // ============================================================
    // PINPOINT / FIELD GEOMETRY  -- SET THESE FOR YOUR ROBOT & FIELD
    // ============================================================
    // ALLIANCE: set true in your RED OpMode, false in your BLUE OpMode.
    // The simplest setup is to have two OpModes that differ only in this one line
    // (e.g. copy this file to TeleopRed/TeleopBlue and flip the flag + @TeleOp name).
    private static final boolean ALLIANCE_IS_RED = true;

    // Goal locations on the field, in the SAME units/origin you initialize the
    // Pinpoint with (this file uses millimeters). Measure from your chosen field
    // origin to each goal. BOTH are TODO -- still 0 until you fill them in.
    private static final double RED_GOAL_X_MM  = 0.0;   // TODO: set red goal X (mm)
    private static final double RED_GOAL_Y_MM  = 0.0;   // TODO: set red goal Y (mm)
    private static final double BLUE_GOAL_X_MM = 0.0;   // TODO: set blue goal X (mm)
    private static final double BLUE_GOAL_Y_MM = 0.0;   // TODO: set blue goal Y (mm)

    // Active goal, chosen by alliance. Used by both auto-aim and hood distance.
    private static final double GOAL_X_MM = ALLIANCE_IS_RED ? RED_GOAL_X_MM : BLUE_GOAL_X_MM;
    private static final double GOAL_Y_MM = ALLIANCE_IS_RED ? RED_GOAL_Y_MM : BLUE_GOAL_Y_MM;

    // ---- Auto-aim P-controller ----
    // Turn power = KP * heading_error(deg), clamped to +/- AIM_MAX_TURN.
    // AIM_DEADBAND_DEG: within this error, stop turning (counts as "aimed").
    // Tune KP up until it turns briskly without oscillating; back off if it wobbles.
    private static final double AIM_KP           = 0.020; // power per degree of error
    private static final double AIM_MAX_TURN     = 0.6;   // cap on auto-turn power
    private static final double AIM_DEADBAND_DEG = 1.5;   // "close enough" threshold

    // Odometry pod offsets from the robot's center of rotation, in mm.
    // X pod (forward pod) is 1 in LEFT of center  -> +25.4 mm
    // Y pod (strafe pod)  is 4.5 in FORWARD of center -> +114.3 mm
    private static final double POD_X_OFFSET_MM = 25.4;
    private static final double POD_Y_OFFSET_MM = 114.3;

    // Tunable power constants
    private static final double INTAKE_POWER    = 1.0;
    private static final double SHOOTER_POWER   = 0.8;
    private static final double SLOW_MODE_SCALE = 0.3;

    // ============================================================
    // FLYWHEEL OPEN-LOOP POWER CONTROL
    // ============================================================
    // We do NOT use the velocity PID. With two motors driving one shared flywheel,
    // two independent velocity PIDs fight each other to a stalemate (they settle at
    // some balance point that does not track the commanded target). Both motors now
    // run open-loop via setPower(), scaled from activeRpm.
    //
    // TICKS_PER_REV is kept only for converting the encoder's ticks/sec back to RPM
    // for telemetry; the flywheels run on raw power, not velocity.
    private static final double TICKS_PER_REV = 28.0;
    private static final double TARGET_RPM    = 5000.0;
    private static final double READY_RPM     = 4700.0; // "ready to shoot" threshold (~95%)

    // ---- Flywheel manual adjust (X button) ----
    // Press X to toggle MANUAL flywheel-speed mode. While ON, dpad up/down changes the
    // active target RPM by FLYWHEEL_MANUAL_STEP_RPM per loop. Dpad is captured by this
    // mode (hood manual cannot use dpad while flywheel manual is on). When OFF, the
    // flywheels stay at whatever RPM the driver left them at -- they do NOT reset.
    // Speed is clamped to [FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM].
    private static final double FLYWHEEL_MANUAL_STEP_RPM = 75.0; // RPM per loop
    private static final double FLYWHEEL_MIN_RPM         = 0.0;
    private static final double FLYWHEEL_MAX_RPM         = 6000.0; // bare motor's physical max

    // Open-loop control uses rpmToPower(activeRpm) to compute the raw power each loop.

    // ============================================================
    // SERVO RANGE
    // ============================================================
    // Servo.setPosition() takes 0.0..1.0, which spans the servo's FULL physical
    // travel. To work in degrees we need to know that full travel.
    //   - Many goBILDA servos: 300 degrees   <-- DEFAULT
    //   - Some are 180 or 270. VERIFY against your servo's spec sheet.
    private static final double SERVO_FULL_RANGE_DEG = 300.0;

    // ---- Hood limits ----
    // 0 deg = fully retracted (servo position 0.0). The hood may only move FORWARD of 0;
    // it is hard-clamped so it can never go backward past 0 in any mode.
    private static final double HOOD_MIN_DEG = 0.0;

    // AUTO mode (distance-based) opens the hood between 0 and HOOD_AUTO_MAX_DEG.
    private static final double HOOD_AUTO_MAX_DEG = 120.0;

    // MANUAL mode forward limit. Set to the full servo range so manual is not capped
    // at 5 deg. Still clamped to >= 0 so it can only travel forward of the start.
    private static final double HOOD_MANUAL_MAX_DEG = SERVO_FULL_RANGE_DEG;

    // ---- Hood distance-based adjustment ----
    // As the robot gets farther from the goal, the hood opens further (more forward),
    // scaling linearly from HOOD_MIN_DEG at NEAR distance to HOOD_AUTO_MAX_DEG at FAR.
    // Units are MILLIMETERS to match the Pinpoint pose. Tune to your field.
    private static final double HOOD_NEAR_DIST = 600.0;  // ~24 in: at/under this -> 0 deg
    private static final double HOOD_FAR_DIST  = 3000.0; // ~118 in: at/over this -> auto max

    // ---- Gate: 11 deg open, 45 deg closed. Toggles on B. ----
    private static final double GATE_OPEN_DEG   = 11.0;
    private static final double GATE_CLOSED_DEG = 45.0;

    // ---- Hood manual override ----
    // Press Y to toggle between AUTO (distance-based) and MANUAL (dpad) hood control.
    // In manual mode, dpad up moves the hood forward, dpad down brings it back toward 0
    // (but never past 0 -- forward-only). No 5-deg cap in manual; limited by HOOD_MANUAL_MAX_DEG.
    // HOOD_MANUAL_STEP is degrees per loop while a dpad button is held.
    private static final double HOOD_MANUAL_STEP = 2.0; // deg per loop (bigger = faster)

    // Gate toggle state. Gate starts OPEN at the beginning, so gateOpen = true.
    private boolean gateOpen = true;
    private boolean prevB    = false;

    // Hood manual-override state
    private boolean hoodManual = false;       // false = auto (distance), true = manual (dpad)
    private boolean prevY      = false;
    private double  hoodManualDeg = HOOD_MIN_DEG; // current commanded angle in manual mode

    // Auto-aim state
    private boolean autoAim = false;  // toggled by A
    private boolean prevA   = false;

    // Flywheel manual-speed state
    private boolean flywheelManual = false;  // toggled by X
    private boolean prevX          = false;
    private double  activeRpm      = TARGET_RPM; // current commanded RPM (persists across mode toggles)

    @Override
    public void runOpMode() {
        // --- Hardware map ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooter   = hardwareMap.get(DcMotor.class, "shooter");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        // --- Pinpoint odometry ---
        // "pinpoint" is the I2C device name in your robot config (must NOT be on port 0).
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Offsets in MILLIMETERS. Older drivers use setOffsets(x, y); newer ones add a
        // DistanceUnit arg: setOffsets(x, y, DistanceUnit.MM). If this line errors,
        // your driver wants the other form -- swap to the commented version below.
        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM);
        // pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        // Use the pod type you actually have. Options include goBILDA_SWINGARM_POD
        // and goBILDA_4_BAR_POD. For a non-goBILDA pod use setEncoderResolution(ticksPerMM).
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        // Reset position + recalibrate IMU. Keep the robot STILL during init.
        pinpoint.resetPosAndIMU();

        // --- Motor directions ---
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel1.setDirection(DcMotorSimple.Direction.FORWARD); // was REVERSE
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE); // was FORWARD

        // --- Zero power behavior ---
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Flywheels use BRAKE so lowering activeRpm actually slows them down quickly,
        // instead of just cutting power and letting the heavy wheel coast for ~10+ seconds.
        // flywheel1 is the MASTER (velocity PID); flywheel2 is the FOLLOWER (gets the
        // same raw power as flywheel1 each loop). Both still need BRAKE so the follower
        // brakes alongside the master when the target drops.
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive motors run without encoders
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Flywheels: open-loop power control. Velocity PID was fighting itself with two
        // motors driving one shared wheel. Both motors now get the SAME raw power scaled
        // from activeRpm. Encoders still work for telemetry (getVelocity reads ticks/sec
        // regardless of mode).
        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --- Servo init ---
        // Gate starts OPEN (11 deg) at the beginning. First B press will CLOSE it.
        gate.setPosition(gateDegToPos(GATE_OPEN_DEG));
        // Hood starts at 0 deg.
        hood.setPosition(hoodDegToPos(HOOD_MIN_DEG));

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        // Spin both flywheels up to the active target. Open-loop: power is just
        // activeRpm / FLYWHEEL_MAX_RPM, sent identically to both motors. Re-commanded
        // every loop so manual changes take effect.
        double startPower = rpmToPower(activeRpm);
        flywheel1.setPower(startPower);
        flywheel2.setPower(startPower);

        while (opModeIsActive()) {
            // Refresh odometry pose once per loop (single bulk I2C read).
            pinpoint.update();

            // -------- Auto-aim toggle (A, rising edge) --------
            boolean aNow = gamepad1.a;
            if (aNow && !prevA) {
                autoAim = !autoAim;
            }
            prevA = aNow;

            // -------- Flywheel manual mode toggle (X, rising edge) --------
            boolean xNow = gamepad1.x;
            if (xNow && !prevX) {
                flywheelManual = !flywheelManual;
            }
            prevX = xNow;

            // While flywheel manual is ON, dpad up/down changes activeRpm.
            // Dpad is "captured" by this mode -- the hood manual logic below checks
            // flywheelManual and ignores dpad while it's true.
            if (flywheelManual) {
                if (gamepad1.dpad_up)   activeRpm += FLYWHEEL_MANUAL_STEP_RPM;
                if (gamepad1.dpad_down) activeRpm -= FLYWHEEL_MANUAL_STEP_RPM;
                activeRpm = Math.max(FLYWHEEL_MIN_RPM, Math.min(FLYWHEEL_MAX_RPM, activeRpm));
            }

            // Command flywheels to the active RPM every loop, so changes take effect.
            // When flywheel manual is OFF, activeRpm is unchanged -- they stay where the
            // driver left them (don't snap back to default).
            // Open-loop: both motors get the same raw power scaled from activeRpm.
            double fwPower = rpmToPower(activeRpm);
            flywheel1.setPower(fwPower);
            flywheel2.setPower(fwPower);

            // -------- Mecanum drive (robot-centric) --------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // When auto-aim is ON, the heading P-controller takes over rotation (rx).
            // Translation (left stick x/y) stays fully under driver control, so you can
            // still drive/strafe while the robot holds its nose on the goal.
            // SAFETY: if goal coords are still the default (0,0), auto-aim refuses to
            // drive rotation -- otherwise the robot would spin trying to face its own
            // starting position. Set real RED/BLUE_GOAL_*_MM values to enable.
            double headingErrDeg = 0.0; // for telemetry
            boolean aimed = false;
            boolean goalConfigured = !(GOAL_X_MM == 0.0 && GOAL_Y_MM == 0.0);
            if (autoAim && goalConfigured) {
                headingErrDeg = aimHeadingErrorDeg();
                if (Math.abs(headingErrDeg) <= AIM_DEADBAND_DEG) {
                    rx = 0.0;
                    aimed = true;
                } else {
                    double turn = AIM_KP * headingErrDeg;
                    // clamp to max turn power
                    turn = Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
                    rx = turn;
                }
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            double slowScale = 1.0 - (gamepad1.right_trigger * (1.0 - SLOW_MODE_SCALE));

            leftFront.setPower(lfPower * slowScale);
            leftBack.setPower(lbPower * slowScale);
            rightFront.setPower(rfPower * slowScale);
            rightBack.setPower(rbPower * slowScale);

            // -------- Intake / Outtake --------
            // Left trigger = OUTTAKE: reverse intake AND shooter together to eject.
            // Left bumper  = intake in.
            // Right bumper = shooter feed forward.
            // Intake and shooter run independently, so left bumper + right bumper at the
            // same time runs intake in AND shooter feed at the same time.
            if (gamepad1.left_trigger > 0.1) {
                // Outtake takes priority: both reverse together, scaled by trigger.
                intake.setPower(-INTAKE_POWER * gamepad1.left_trigger);
                shooter.setPower(-SHOOTER_POWER * gamepad1.left_trigger);
            } else {
                // Intake and shooter controlled independently.
                intake.setPower(gamepad1.left_bumper ? INTAKE_POWER : 0.0);
                shooter.setPower(gamepad1.right_bumper ? SHOOTER_POWER : 0.0);
            }

            // -------- Hood: AUTO (distance) or MANUAL (dpad) --------
            // Y toggles the mode (rising edge).
            boolean yNow = gamepad1.y;
            if (yNow && !prevY) {
                hoodManual = !hoodManual;
            }
            prevY = yNow;

            double distance = getDistanceToGoal();
            double hoodDeg;
            if (hoodManual) {
                // dpad up = forward, dpad down = back toward 0 (but never below 0).
                // BUT: flywheel manual mode takes priority on dpad. While it's on, the
                // hood holds its current angle and ignores dpad presses.
                if (!flywheelManual) {
                    if (gamepad1.dpad_up)   hoodManualDeg += HOOD_MANUAL_STEP;
                    if (gamepad1.dpad_down) hoodManualDeg -= HOOD_MANUAL_STEP;
                }
                // Clamp to [0, manual max]: forward-only (>=0), uncapped by the 5-deg auto limit.
                hoodManualDeg = Math.max(HOOD_MIN_DEG, Math.min(HOOD_MANUAL_MAX_DEG, hoodManualDeg));
                hoodDeg = hoodManualDeg;
            } else {
                hoodDeg = hoodAngleForDistance(distance);
            }
            hood.setPosition(hoodDegToPos(hoodDeg));

            // -------- Gate: toggle open/closed on B (rising edge) --------
            boolean bNow = gamepad1.b;
            if (bNow && !prevB) {
                gateOpen = !gateOpen;
                gate.setPosition(gateDegToPos(gateOpen ? GATE_OPEN_DEG : GATE_CLOSED_DEG));
            }
            prevB = bNow;

            // -------- Telemetry --------
            double fw1Ticks = flywheel1.getVelocity();
            double fw2Ticks = flywheel2.getVelocity();
            double fw1Rpm = (fw1Ticks / TICKS_PER_REV) * 60.0;
            double fw2Rpm = (fw2Ticks / TICKS_PER_REV) * 60.0;
            double avgRpm = (fw1Rpm + fw2Rpm) / 2.0;
            boolean ready = avgRpm >= READY_RPM;

            telemetry.addData("Drive (y,x,rx)", "%.2f, %.2f, %.2f", y, x, rx);
            telemetry.addData("Slow scale", "%.2f", slowScale);
            telemetry.addData("Intake power", "%.2f", intake.getPower());
            telemetry.addData("Shooter power", "%.2f", shooter.getPower());
            telemetry.addLine();
            telemetry.addData("Shoot ready?", ready ? ">>> READY <<<" : "spinning up...");
            telemetry.addData("FW1 RPM (actual)", "%.0f", fw1Rpm);
            telemetry.addData("FW2 RPM (actual)", "%.0f", fw2Rpm);
            telemetry.addData("Avg RPM", "%.0f", avgRpm);
            telemetry.addData("FW power commanded", "%.2f", rpmToPower(activeRpm));
            telemetry.addData("Target RPM", "%.0f (ready at %.0f)", activeRpm, READY_RPM);
            telemetry.addData("FW mode", flywheelManual ? "MANUAL (X on, dpad adjusts)" : "fixed (press X to adjust)");
            telemetry.addLine();
            telemetry.addData("Alliance", ALLIANCE_IS_RED ? "RED" : "BLUE");
            String aimStatus;
            if (!autoAim)            aimStatus = "OFF (press A)";
            else if (!goalConfigured) aimStatus = "BLOCKED (goal coords not set)";
            else if (aimed)           aimStatus = ">>> AIMED <<<";
            else                      aimStatus = "turning...";
            telemetry.addData("Auto-aim", aimStatus);
            telemetry.addData("Heading err", "%.1f deg", headingErrDeg);
            telemetry.addData("Stick rx (raw)", "%.3f", gamepad1.right_stick_x);
            telemetry.addLine();
            // Robot pose (used to find goal coords -- see method A in the docs).
            // Push the robot to the goal and read these numbers; they ARE your goal coords.
            Pose2D pose = pinpoint.getPosition();
            double robotX = pose.getX(DistanceUnit.MM);
            double robotY = pose.getY(DistanceUnit.MM);
            double robotHeading = pose.getHeading(AngleUnit.DEGREES);
            telemetry.addLine();
            telemetry.addData("Robot X (mm)", "%.0f", robotX);
            telemetry.addData("Robot Y (mm)", "%.0f", robotY);
            telemetry.addData("Robot heading (deg)", "%.1f", robotHeading);
            telemetry.addData("Distance to goal", "%.0f mm", distance);
            telemetry.addData("Hood mode", hoodManual ? "MANUAL (dpad up/down)" : "AUTO (distance)");
            telemetry.addData("Hood angle", "%.1f deg", hoodDeg);
            telemetry.addData("Hood servo pos", "%.4f", hood.getPosition());
            telemetry.addData("Gate", gateOpen ? "OPEN (11 deg)" : "CLOSED (45 deg)");
            telemetry.update();
        }

        stopAllMotors();
    }

    // ============================================================
    // DISTANCE TO GOAL -- from Pinpoint odometry
    // ============================================================
    // Reads the latest fused pose and returns straight-line distance to the goal.
    // pinpoint.update() must have been called this loop (it is, at the top of the loop).
    // Returns millimeters, matching GOAL_X_MM / GOAL_Y_MM and the HOOD_*_DIST constants.
    private double getDistanceToGoal() {
        Pose2D pose = pinpoint.getPosition();
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double dx = GOAL_X_MM - rx;
        double dy = GOAL_Y_MM - ry;
        return Math.hypot(dx, dy);
    }

    // ============================================================
    // AUTO-AIM: heading error to the goal, in degrees
    // ============================================================
    // Returns how far (deg) the robot must rotate to face the goal.
    // Positive = need to turn counter-clockwise (left), negative = clockwise (right),
    // matching the sign convention of rx in the mecanum mix above. Wrapped to [-180,180]
    // so the robot always turns the short way around.
    //
    // Assumes the Pinpoint heading and the field frame are consistent: heading is the
    // angle of the robot's +X (forward) axis measured CCW from the field +X axis, and
    // the goal coords are in that same field frame. This matches goBILDA's convention
    // (forward increases X, left increases Y, CCW-positive heading). If your robot aims
    // 90 or 180 deg off, your start heading or field origin is rotated -- see notes.
    private double aimHeadingErrorDeg() {
        Pose2D pose = pinpoint.getPosition();
        double rx = pose.getX(DistanceUnit.MM);
        double ry = pose.getY(DistanceUnit.MM);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        double dx = GOAL_X_MM - rx;
        double dy = GOAL_Y_MM - ry;
        double targetDeg = Math.toDegrees(Math.atan2(dy, dx));

        return wrapDeg(targetDeg - headingDeg);
    }

    // Convert a target RPM to raw motor power (0..1) for open-loop flywheel control.
    // Linear scale: activeRpm / FLYWHEEL_MAX_RPM. Clamped to [0, 1].
    private double rpmToPower(double rpm) {
        double p = rpm / FLYWHEEL_MAX_RPM;
        return Math.max(0.0, Math.min(1.0, p));
    }

    // Wrap an angle in degrees to the range [-180, 180].
    private double wrapDeg(double deg) {
        while (deg >  180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    // Linear map: HOOD_NEAR_DIST -> HOOD_MIN_DEG, HOOD_FAR_DIST -> HOOD_AUTO_MAX_DEG.
    // Used only in AUTO mode. Clamped so it stays within 0..auto-max.
    private double hoodAngleForDistance(double distance) {
        if (distance <= HOOD_NEAR_DIST) return HOOD_MIN_DEG;
        if (distance >= HOOD_FAR_DIST)  return HOOD_AUTO_MAX_DEG;
        double t = (distance - HOOD_NEAR_DIST) / (HOOD_FAR_DIST - HOOD_NEAR_DIST);
        return HOOD_MIN_DEG + t * (HOOD_AUTO_MAX_DEG - HOOD_MIN_DEG);
    }

    // Convert a hood angle (deg) to a servo position. Hard safety clamp: forward-only
    // (never below 0 deg) and never beyond the servo's full physical range. This is the
    // floor guarantee that the hood cannot be driven backward past its 0 start, in any mode.
    private double hoodDegToPos(double deg) {
        double clamped = Math.max(HOOD_MIN_DEG, Math.min(SERVO_FULL_RANGE_DEG, deg));
        return clamped / SERVO_FULL_RANGE_DEG;
    }

    // Convert a gate angle (deg) to a servo position over the full servo range.
    private double gateDegToPos(double deg) {
        double clamped = Math.max(0.0, Math.min(SERVO_FULL_RANGE_DEG, deg));
        return clamped / SERVO_FULL_RANGE_DEG;
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    private void stopAllMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        intake.setPower(0);
        shooter.setPower(0);
        flywheel1.setPower(0);
        flywheel2.setPower(0);
    }
}