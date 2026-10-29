package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test Code")
public class BlockTest extends LinearOpMode {

    // ---- Tunable timing (seconds) ----
    public static double BEAM_CONFIRM_TIME = 0.35; // beam must be true this long before lifting intake
    public static double GATE_OPEN_DELAY = 0.6;    // after lifting intake, wait this long, then open gate

    // ---- Shoot sequence (right trigger) ----
    public static double SHOOT_FEED_TIME = 0.4;     // feed this long, then stop intake and close gate
    public static double SHOOT_LOWER_DELAY = 0.2;   // after stopping, wait this long, then lower intake + resume intaking
    public static double SHOOT_FEED_POWER = 0.7;    // intake power while feeding (70%)
    public static double RESUME_INTAKE_POWER = 1.0; // same as the X button

    // ---- Flywheel ----
    public static int VELOCITY_TRIM_STEP = 10;      // ticks per bumper press

    private DcMotor intakeMotorFront;
    private DcMotor shooter;
    private DcMotor frontRight;
    private DcMotor rearRight;
    private DcMotor frontLeft;
    private DcMotor rearLeft;
    private Servo transferGate;
    private Servo intakePivot;
    private DcMotor intakeMotorBack;
    private Servo LEDIndicator;
    private Servo hood;
    private DigitalChannel beamBreak;
    private Servo flowerPivot;

    @Override
    public void runOpMode() {
        int velocity;
        float forward;
        float strafe;
        float rotate;

        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        transferGate = hardwareMap.get(Servo.class, "transferGate");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");
        LEDIndicator = hardwareMap.get(Servo.class, "LEDIndicator");
        hood = hardwareMap.get(Servo.class, "hood");
        beamBreak = hardwareMap.get(DigitalChannel.class, "beamBreak");
        flowerPivot = hardwareMap.get(Servo.class, "flowerPivot");

        // Beam break auto-sequence state
        ElapsedTime beamTimer = new ElapsedTime();     // how long beam has been continuously true
        ElapsedTime sequenceTimer = new ElapsedTime(); // time since intake was lifted
        boolean sequenceStarted = false;
        boolean gateOpened = false;

        // Shoot sequence state
        ElapsedTime shootTimer = new ElapsedTime();
        boolean shooting = false;
        int shootStage = 0; // 1 = feeding, 2 = stopped, waiting to lower

        // Flywheel state
        boolean flywheelOn = false;
        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;

        waitForStart();
        intakeMotorFront.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (opModeIsActive()) {
            velocity = 2220;
            transferGate.setPosition(0.5);
            while (opModeIsActive()) {
                forward = gamepad1.right_stick_y;
                strafe = -gamepad1.right_stick_x;
                rotate = gamepad1.left_stick_x;
                frontLeft.setPower(forward + (strafe - rotate));
                frontRight.setPower(forward - (strafe - rotate));
                rearLeft.setPower(forward - (strafe + rotate));
                rearRight.setPower(forward + strafe + rotate);
                if (gamepad1.dpad_up) {
                    intakePivot.setPosition(0.87);
                }
                if (gamepad1.dpad_down) {
                    intakePivot.setPosition(0.69);
                }
                if (gamepad1.x) {
                    intakeMotorBack.setPower(1);
                    intakeMotorFront.setPower(1);
                }
                if (gamepad1.y) {
                    intakeMotorBack.setPower(0);
                    intakeMotorFront.setPower(0);
                }
                if (gamepad1.left_trigger_pressed) {
                    intakeMotorBack.setPower(-1);
                    intakeMotorFront.setPower(-1);
                }
                if (gamepad1.b) {
                    intakeMotorBack.setPower(0.5);
                    intakeMotorFront.setPower(0.5);
                }
                // Right trigger starts the shoot sequence (only if one isn't already running)
                if (gamepad1.right_trigger_pressed && !shooting) {
                    shooting = true;
                    shootStage = 1;
                    shootTimer.reset();
                    intakeMotorBack.setPower(SHOOT_FEED_POWER);
                    intakeMotorFront.setPower(SHOOT_FEED_POWER);
                }

                // ---- Flywheel controls ----
                // Dpad right = flywheel on, dpad left = flywheel off
                if (gamepad1.dpad_right) {
                    flywheelOn = true;
                }
                if (gamepad1.dpad_left) {
                    flywheelOn = false;
                }

                // Bumpers trim target velocity, one step per press
                if (gamepad1.right_bumper && !lastRightBumper) {
                    velocity += VELOCITY_TRIM_STEP;
                }
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    velocity -= VELOCITY_TRIM_STEP;
                    if (velocity < 0) {
                        velocity = 0;
                    }
                }
                lastRightBumper = gamepad1.right_bumper;
                lastLeftBumper = gamepad1.left_bumper;

                if (gamepad1.left_stick_button) {
                    velocity += -2;
                }
                if (gamepad1.right_stick_button) {
                    velocity += 2;
                }

                // Apply flywheel state (runs every loop so trims take effect live)
                if (flywheelOn) {
                    ((DcMotorEx) shooter).setVelocity(velocity);
                } else {
                    ((DcMotorEx) shooter).setVelocity(0);
                }

                // ---- Shoot sequence ----
                if (shooting) {
                    if (shootStage == 1) {
                        // Feeding
                        intakeMotorBack.setPower(SHOOT_FEED_POWER);
                        intakeMotorFront.setPower(SHOOT_FEED_POWER);
                        if (shootTimer.seconds() >= SHOOT_FEED_TIME) {
                            intakeMotorBack.setPower(0);
                            intakeMotorFront.setPower(0);
                            transferGate.setPosition(0.5); // close transfer gate
                            shootStage = 2;
                            shootTimer.reset();
                        }
                    } else if (shootStage == 2) {
                        // Intake stopped, waiting to lower
                        intakeMotorBack.setPower(0);
                        intakeMotorFront.setPower(0);
                        if (shootTimer.seconds() >= SHOOT_LOWER_DELAY) {
                            intakePivot.setPosition(0.69); // lower intake
                            intakeMotorBack.setPower(RESUME_INTAKE_POWER);
                            intakeMotorFront.setPower(RESUME_INTAKE_POWER);
                            shooting = false;
                            shootStage = 0;

                            // Re-arm the beam break sequence for the next piece
                            sequenceStarted = false;
                            gateOpened = false;
                            beamTimer.reset();
                        }
                    }
                }

                // ---- Beam break auto-sequence (paused during the shoot sequence) ----
                boolean beamTrue = beamBreak.getState();

                if (shooting) {
                    beamTimer.reset();
                } else {
                    if (!beamTrue && !sequenceStarted) {
                        // Beam not true: keep the confirm timer reset
                        beamTimer.reset();
                    }

                    // Step 1: beam true long enough -> lift intake and stop intake
                    if (!sequenceStarted && beamTrue && beamTimer.seconds() >= BEAM_CONFIRM_TIME) {
                        intakePivot.setPosition(0.87);
                        intakeMotorBack.setPower(0);
                        intakeMotorFront.setPower(0);
                        sequenceStarted = true;
                        gateOpened = false;
                        sequenceTimer.reset();
                    }

                    // Step 2: after the delay -> open transfer gate
                    if (sequenceStarted && !gateOpened && sequenceTimer.seconds() >= GATE_OPEN_DELAY) {
                        transferGate.setPosition(0.65);
                        gateOpened = true;
                    }

                    // Re-arm once the sequence is done and the beam clears
                    if (gateOpened && !beamTrue) {
                        sequenceStarted = false;
                        gateOpened = false;
                        beamTimer.reset();
                    }
                }

                LEDIndicator.setPosition(0.5);
                hood.setPosition(0.7);
                telemetry.addData("flywheel on", flywheelOn);
                telemetry.addData(" target velocity", velocity);
                telemetry.addData("actual velocity", ((DcMotorEx) shooter).getVelocity());
                telemetry.addData("beam", beamTrue);
                telemetry.addData("shooting", shooting);
                telemetry.update();
            }
        }
    }
}