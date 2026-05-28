package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.Flywheel.FlywheelClass;

@TeleOp(name = "Decode Teleop", group = "Teleop")
public class DustinCoolCode extends LinearOpMode {

    // Drive motors
    private DcMotor leftFront, leftBack, rightFront, rightBack;

    // Intake / shooter motors
    private DcMotor intake;     // lower two wheels (one motor)
    private DcMotor shooter;    // top wheel, pushes ball into flywheels
    private DcMotor flywheel1;
    private DcMotor flywheel2;
    FlywheelClass flywheel;
    private double tgtVEL = 250;
    // Tunable power constants
    private static final double INTAKE_POWER   = 1.0;
    private static final double SHOOTER_POWER  = 1.0;
    private static final double FLYWHEEL_POWER = 1.0;
    private static final double SLOW_MODE_SCALE = 0.3;

    @Override
    public void runOpMode() {
        // --- Hardware map ---
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intake    = hardwareMap.get(DcMotor.class, "intake");
        shooter   = hardwareMap.get(DcMotor.class, "shooter");
        flywheel = new FlywheelClass(hardwareMap);
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");

        // --- Motor directions ---
        // Left side drive motors spin counter-clockwise (forward), right side opposite
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Intake and shooter both need clockwise (FORWARD by default on most motors)
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // Counter-rotating flywheels
        flywheel1.setDirection(DcMotorSimple.Direction.REVERSE); // clockwise
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE); // counter-clockwise

        // --- Zero power behavior ---
        // Brake the drive motors so the robot stops crisply
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Let intake / shooter / flywheels coast when unpowered
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Run without encoders — set to RUN_USING_ENCODER if you want velocity control later
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        // Spin flywheels up at the start and leave them running
        flywheel1.setPower(FLYWHEEL_POWER);
        flywheel2.setPower(FLYWHEEL_POWER);

        while (opModeIsActive()) {
            // -------- Mecanum drive (robot-centric) --------
            double y  = -gamepad1.left_stick_y; // forward/back (stick Y is inverted)
            double x  =  gamepad1.left_stick_x; // strafe
            double rx =  gamepad1.right_stick_x; // rotate

            // Normalize so no value exceeds 1.0
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            // Slow mode: right trigger scales drive power down
            double slowScale = 1.0 - (gamepad1.right_trigger * (1.0 - SLOW_MODE_SCALE));
            // when trigger == 0 -> scale = 1.0 (full); when trigger == 1 -> scale = SLOW_MODE_SCALE

            leftFront.setPower(lfPower * slowScale);
            leftBack.setPower(lbPower * slowScale);
            rightFront.setPower(rfPower * slowScale);
            rightBack.setPower(rbPower * slowScale);

            // -------- Intake (lower two wheels) --------
            // Left bumper = intake in, left trigger = outtake (reverse)
            // Trigger wins if both are pressed
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-INTAKE_POWER * gamepad1.left_trigger);
            } else if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0.0);
            }

            // -------- Shooter feed (top wheel) --------
            // Right bumper held -> push ball into flywheels
            shooter.setPower(gamepad1.right_bumper ? SHOOTER_POWER : 0.0);

            // -------- Telemetry --------
            telemetry.addData("Drive (y,x,rx)", "%.2f, %.2f, %.2f", y, x, rx);
            telemetry.addData("Slow scale", "%.2f", slowScale);
            telemetry.addData("Intake power", intake.getPower());
            telemetry.addData("Shooter power", shooter.getPower());
            telemetry.addData("Flywheels", "running at %.2f", FLYWHEEL_POWER);
            telemetry.addData("tgtVEL:", tgtVEL);
            telemetry.addData("Current Vel:", flywheel.getVelocityRadPerSec());
            telemetry.addData("ESTIMATED RPM:", tgtVEL * 9.549297);
            telemetry.update();
        }

        // Stop everything when opmode ends
        stopAllMotors();
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