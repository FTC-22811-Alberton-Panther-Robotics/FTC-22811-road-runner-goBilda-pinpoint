/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * {@code GeminiTeleop} is an experimental TeleOp program initially generated by Gemini and customized
 * by Nathan Morris for the Alberton Panther Robotics FTC Team #22811.
 * This code is designed for the TeleOp period of an FTC competition and is optimized for
 * efficient and precise control of the robot.
 *
 * <p><b>Key Features:</b></p>
 * <ul>
 *     <li><b>Mecanum Drivetrain:</b> Field-centric Mecanum drive for omnidirectional movement.</li>
 *     <li><b>Linear Lift:</b> Precise vertical lift mechanism with encoder feedback and velocity control.</li>
 *     <li><b>Intake System:</b> Continuous rotation servo for collecting and depositing game elements.</li>
 *     <li><b>Arm Control:</b> Servo-controlled arm for various positioning needs.</li>
 *     <li><b>Claw Mechanism:</b> Servo-controlled claw for grasping game elements.</li>
 *     <li><b>Slide Mechanism:</b> Motor controlled horizontal motion to extend and retract.</li>
 *     <li><b>IMU Integration:</b> Field-centric driving for intuitive movement relative to the field.</li>
 *     <li><b>Preset States:</b> Automated sequences for common tasks like scoring and grabbing.</li>
 *     <li><b>Current Monitoring:</b> Alerts and gamepad rumble for current limit exceedances.</li>
 *     <li><b>Driver Control:</b> User-friendly controls via two gamepads.</li>
 * </ul>
 *
 * <p><b>Hardware */

@TeleOp(name = "Bozeman Teleop", group = "Competition")
//@Disabled
public class BozemanTeleop extends LinearOpMode {

    // Constants
    private static final double LIFT_TICKS_PER_MM = 28 * 12 / 120.0; // RevRobotics 28 ticks/rev motor, with 12:1 gear reduction, and belt travel of 120mm/rev
    private static final double LIFT_VELOCITY = 2100;
    private static final double LIFT_COLLAPSED_INTO_ROBOT = 0;
    private static final double LIFT_READY_TO_SCORE_SPECIMEN = 145 * LIFT_TICKS_PER_MM;
    private static final double LIFT_SCORE_SPECIMEN = 300 * LIFT_TICKS_PER_MM;
    private static final double LIFT_MAX_HEIGHT = 600 * LIFT_TICKS_PER_MM;
    private static final double INTAKE_COLLECT = -1.0;
    private static final double INTAKE_OFF = 0.0;
    private static final double INTAKE_DEPOSIT = 0.5;
    private static final double SLIDE_TICKS_PER_MM = 28 * 12 / 120.0; // RevRobotics 28 ticks/rev motor, with 12:1 gear reduction, and belt travel of 120mm/rev
    private static final double SLIDE_HALF_OUT = 150 * SLIDE_TICKS_PER_MM;
    private static final double SLIDE_FULLY_EXTENDED = 300 * SLIDE_TICKS_PER_MM; // TODO: Example value, replace with actual value
    private static final double SLIDE_COLLAPSED_INTO_ROBOT = 0;
    private static final double SLIDE_VELOCITY = 2100;
    private static final double ARM_SCORE_SPECIMEN = .33;
    private static final double ARM_GRAB_SPECIMEN = 1;
    private static final double ARM_TRANSFER = 1; // TODO: Example value, replace with actual value
    private static final double CLAW_OPEN = 0;
    private static final double CLAW_CLOSED = 1;
    private static final double WRIST_UP = .5; // TODO: Example value, replace with actual value
    private static final double WRIST_DOWN = .9; // TODO: Example value, replace with actual value
    private static final double WRIST_FOLDED =0; // TODO: Example value, replace with actual value
    private static final double ACTUATORS_COLLAPSED_INTO_ROBOT = 0;
    private static final double ACTUATORS_HANG = 200; // TODO: Example value, replace with actual value
    private static final double ACTUATORS_FULLY_EXTENDED = 300; // TODO: Example value, replace with actual value
    private static final double ACTUATORS_VELOCITY = 2100;

    // Enums
    private enum PresetState {
        SCORE_SPECIMEN, READY_TO_SCORE_SPECIMEN, GRAB_SPECIMEN,CLEAR_BARRIER, TRANSFER, COLLECT, FOLD_IN_LOWER, SLIDE_OUT_MANUAL,SLIDE_HALF_OUT, SLIDE_FULL_OUT, INTAKE,SLIDE_IN_MANUAL, COLLECTION_SUCCESSFUL, PRE_HANG, HANG, IDLE
    }

    // Hardware
    private DcMotor rightActuator = null;
    private DcMotor leftActuator = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor liftMotor = null;
    private DcMotor slideMotor = null;
    private Servo   armServo = null;
    private Servo   upperClawServo = null;
    private Servo   lowerClawServo = null;
    private Servo   wristServo = null;
    private IMU     imu = null;

    // State
    private PresetState currentPresetState = PresetState.FOLD_IN_LOWER;
    private double liftTargetPosition = LIFT_COLLAPSED_INTO_ROBOT;
    private double slideTargetPosition = SLIDE_COLLAPSED_INTO_ROBOT;
    private double leftActuatorTargetPosition = ACTUATORS_COLLAPSED_INTO_ROBOT;
    private double rightActuatorTargetPosition = ACTUATORS_COLLAPSED_INTO_ROBOT;
    private ElapsedTime sequenceTimer = new ElapsedTime();

    // Rumble
    private ElapsedTime rumbleTimer = new ElapsedTime();
    private boolean isRumbling = false;

    // Timing
    private double cycleTime = 0;
    private double loopTime = 0;
    private double oldTime = 0;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        initializeHardware();

        // Wait for the game to start (driver presses PLAY)
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();

        // Reset the IMU yaw angle to zero on start
        resetIMU();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Preset Selection
            if (gamepad1.y) {
                currentPresetState = PresetState.SCORE_SPECIMEN;
            } else if (gamepad1.a){
                currentPresetState = PresetState.GRAB_SPECIMEN;
            } else if (gamepad1.b){
                currentPresetState = PresetState.READY_TO_SCORE_SPECIMEN;
            } else if (gamepad2.y) {
                currentPresetState = PresetState.FOLD_IN_LOWER;
            } else if (gamepad2.x) {
                currentPresetState = PresetState.SLIDE_HALF_OUT;
                sequenceTimer.reset();
            } else if (gamepad2.b){
                currentPresetState = PresetState.SLIDE_FULL_OUT;
                sequenceTimer.reset();
            } else if (gamepad2.a) {
                currentPresetState = PresetState.COLLECT;
            }

            // Update the preset state based on the current state
            updatePresetState(currentPresetState);

            // Drive the robot based on gamepad input
            double y = -squareInputWithSign(gamepad1.left_stick_y); // left stick is negative when up so flip that with negative sign
            double x = squareInputWithSign(gamepad1.left_stick_x);
            double rx = squareInputWithSign(gamepad1.right_stick_x);
            driveRobot(y, x, rx);

            // Reset the IMU yaw angle to zero if the options button is pressed
            if (gamepad1.options) {
                resetIMU();
            }



            // Control the  upper claw based on gamepad input and presets
            controlUpperClaw();

            // Control the lower claw based on gamepad input and presets
            controlLowerClaw();


            // Control the lift based on gamepad input and presets
            controlLift();

            // Control the slide based on gamepad input and presets
            controlSlide();

            // Control the arm based on gamepad input and presets
            controlArm();

            // Control the left and right actuators based on gamepad input and presets
            controlActuators();



            // Update the loop time and cycle time
            updateTiming();

            // Update the telemetry
            updateTelemetry();
        }
    }

    /**
     * Initializes all the hardware components.
     */
    private void initializeHardware() {
        // Motors
        rightActuator = hardwareMap.dcMotor.get("right_actuator");
        leftActuator = hardwareMap.dcMotor.get("left_actuator");
        leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");
        leftRearDrive = hardwareMap.dcMotor.get("left_rear_drive");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        rightRearDrive = hardwareMap.dcMotor.get("right_rear_drive");
        liftMotor = hardwareMap.dcMotor.get("lift");
        slideMotor = hardwareMap.dcMotor.get("slide");

        // Servos
        armServo = hardwareMap.get(Servo.class, "arm");
        upperClawServo = hardwareMap.get(Servo.class, "upperClaw");
        lowerClawServo = hardwareMap.get(Servo.class, "lowerClaw");
        wristServo = hardwareMap.get(Servo.class, "wrist");

        // Motor Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Zero Power Behavior
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Current Limits
        ((DcMotorEx) liftMotor).setCurrentAlert(5, CurrentUnit.AMPS);
        ((DcMotorEx) slideMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        // Lift Motor Setup
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Slide Motor Setup
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Lower Claw Servo Initial State
        wristServo.setPosition(WRIST_FOLDED);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }

    /**
     * Resets the IMU's yaw angle to zero.
     */
    private void resetIMU() {
        imu.resetYaw();
    }

    /**
     * Squares the input value while preserving its sign.
     * This is used to reduce sensitivity at low joystick values.
     *
     * @param input The input value from the joystick.
     * @return The squared input value with the original sign.
     */
    private double squareInputWithSign(double input) {
        return Math.copySign(input * input, input);
    }

    /**
     * Drives the robot based on gamepad input.
     *
     * @param y  The forward/backward movement input.
     * @param x  The left/right movement input.
     * @param rx The rotation input.
     */
    private void driveRobot(double y, double x, double rx) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor position (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontDrive.setPower(frontLeftPower);
        leftRearDrive.setPower(backLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        rightRearDrive.setPower(backRightPower);
    }



    /**
     * Controls the upper claw servo based on gamepad button presses.
     */
    private void controlUpperClaw() {
        if (gamepad1.left_bumper) {
            upperClawServo.setPosition(CLAW_OPEN);
        } else if (gamepad1.right_bumper) {
            upperClawServo.setPosition(CLAW_CLOSED);
        }
    }


    /**
     * Controls the lower claw servo based on gamepad button presses.
     */
    private void controlLowerClaw() {
        if (gamepad2.left_bumper) {
            lowerClawServo.setPosition(CLAW_OPEN);
        } else if (gamepad2.right_bumper) {
            lowerClawServo.setPosition(CLAW_CLOSED);
        }
    }



    /**
     * Controls the lift motor's movement and position.
     */
    private void controlLift() {
        // Manual Lift Control
        if (gamepad1.dpad_up){
            liftTargetPosition += LIFT_VELOCITY * cycleTime;
        } else if (gamepad1.dpad_down){
            liftTargetPosition -= LIFT_VELOCITY * cycleTime;
        }
        if (liftTargetPosition < LIFT_COLLAPSED_INTO_ROBOT) {
            liftTargetPosition = LIFT_COLLAPSED_INTO_ROBOT;
        } else if (liftTargetPosition > LIFT_MAX_HEIGHT) {
            liftTargetPosition = LIFT_MAX_HEIGHT;
        }

        // Set the target position and velocity
        liftMotor.setTargetPosition((int) (liftTargetPosition));
        ((DcMotorEx) liftMotor).setVelocity(LIFT_VELOCITY);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Check for current limit
        if (((DcMotorEx) liftMotor).isOverCurrent()) {
            telemetry.addLine("LIFT MOTOR EXCEEDED CURRENT LIMIT!");
            liftTargetPosition = liftMotor.getCurrentPosition();
            controlRumble(gamepad2, true);
        } else {
            controlRumble(gamepad2, false);
        }
    }

    /**
     * Controls the slide motor's movement and position.
     */
    private void controlSlide() {
        // Manual Slide Control
        slideTargetPosition += (gamepad2.right_trigger - gamepad2.left_trigger) * SLIDE_VELOCITY * cycleTime;
        if (slideTargetPosition < SLIDE_COLLAPSED_INTO_ROBOT) {
            slideTargetPosition = SLIDE_COLLAPSED_INTO_ROBOT;
        } else if (slideTargetPosition > SLIDE_FULLY_EXTENDED) {
            slideTargetPosition = SLIDE_FULLY_EXTENDED;
        }

        // Set the target position and velocity
        slideMotor.setTargetPosition((int) (slideTargetPosition));
        ((DcMotorEx) slideMotor).setVelocity(SLIDE_VELOCITY);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Check for current limit
        if (((DcMotorEx) slideMotor).isOverCurrent()) {
            telemetry.addLine("SLIDE MOTOR EXCEEDED CURRENT LIMIT!");
            slideTargetPosition = slideMotor.getCurrentPosition();
            controlRumble(gamepad2, true);
        } else {
            controlRumble(gamepad2, false);
        }
    }

    /**n
     * Controls the arm servo's position.
     */
    private void controlArm() {
        //armServo.setPosition(armServo.getPosition() + (gamepad1.right_trigger - gamepad1.left_trigger) * .1);
    }

    /**
     * Controls the left and right actuator motors' movements and positions.
     */
    private void controlActuators() {
        // Manual Slide Control
        leftActuatorTargetPosition -= gamepad2.left_stick_y * ACTUATORS_VELOCITY * cycleTime; // negative because left stick is negative when up
        rightActuatorTargetPosition += gamepad2.right_stick_y * ACTUATORS_VELOCITY * cycleTime;

        // Set the target position and velocity
        leftActuator.setTargetPosition((int) leftActuatorTargetPosition);
        rightActuator.setTargetPosition((int) rightActuatorTargetPosition);
        ((DcMotorEx) leftActuator).setVelocity(ACTUATORS_VELOCITY);
        ((DcMotorEx) rightActuator).setVelocity(ACTUATORS_VELOCITY);
        leftActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightActuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Check for current limit
        if (((DcMotorEx) leftActuator).isOverCurrent()) {
            telemetry.addLine("LEFT ACTUATOR EXCEEDED CURRENT LIMIT!");
            leftActuator.setTargetPosition(leftActuator.getCurrentPosition());
            controlRumble(gamepad2, true);
        } else {
            controlRumble(gamepad2, false);
        }
        if (((DcMotorEx) rightActuator).isOverCurrent()) {
            telemetry.addLine("RIGHT ACTUATOR EXCEEDED CURRENT LIMIT!");
            rightActuator.setTargetPosition(rightActuator.getCurrentPosition());
            controlRumble(gamepad2, true);
        } else {
            controlRumble(gamepad2, false);
        }
    }

    /**
     * Calculates and updates the `cycleTime` and `loopTime` variables.
     */
    private void updateTiming() {
        double newTime = getRuntime();
        cycleTime = newTime - oldTime;
        loopTime = newTime - loopTime;
        oldTime = newTime;
    }

    /**
     * Sends data to the driver station telemetry.
     */
    private void updateTelemetry() {
        telemetry.addData("Lift Target", liftMotor.getTargetPosition());
        telemetry.addData("Lift Encoder:", liftMotor.getCurrentPosition());
        telemetry.addData("Lift mm", liftMotor.getCurrentPosition() / LIFT_TICKS_PER_MM);
        telemetry.addData("Preset State", currentPresetState);
        telemetry.addData("Slide Target", slideMotor.getTargetPosition());
        telemetry.addData("Slide Encoder:", slideMotor.getCurrentPosition());
        telemetry.addData("Slide mm", slideMotor.getCurrentPosition() / SLIDE_TICKS_PER_MM);
        telemetry.addData("Arm", armServo.getPosition());
        telemetry.addData("Claw", "Position: %.2f", armServo.getPosition());
        telemetry.addData("Left Actuator", leftActuator.getCurrentPosition());
        telemetry.addData("Right Actuator", rightActuator.getCurrentPosition());
        telemetry.addData("Cycle Time", "%.4f seconds", cycleTime);
        telemetry.update();
    }

    /**
     * Controls the arm's position based on the `PresetState` enum.
     *
     * @param state The current preset state.
     */
    private void updatePresetState(PresetState state) {
        switch (state) {
            case READY_TO_SCORE_SPECIMEN:
                liftTargetPosition = LIFT_READY_TO_SCORE_SPECIMEN;
                armServo.setPosition(ARM_SCORE_SPECIMEN);
                currentPresetState = PresetState.IDLE;
                break;
            case SCORE_SPECIMEN:
                liftTargetPosition = LIFT_SCORE_SPECIMEN;
                currentPresetState = PresetState.IDLE;
                break;
            case GRAB_SPECIMEN:
                liftTargetPosition = LIFT_COLLAPSED_INTO_ROBOT;
                armServo.setPosition(ARM_GRAB_SPECIMEN);
                currentPresetState = PresetState.IDLE;
                break;
            case TRANSFER:
                liftTargetPosition = LIFT_COLLAPSED_INTO_ROBOT;
                armServo.setPosition(ARM_TRANSFER);
                upperClawServo.setPosition(CLAW_OPEN);
                currentPresetState = PresetState.IDLE;
                break;
            case CLEAR_BARRIER:
                wristServo.setPosition(WRIST_UP);
                currentPresetState = PresetState.IDLE;
                break;
            case SLIDE_FULL_OUT:
                wristServo.setPosition(WRIST_UP);
                slideTargetPosition = SLIDE_FULLY_EXTENDED;
                currentPresetState = PresetState.IDLE;
                break;
            case SLIDE_HALF_OUT:
                wristServo.setPosition(WRIST_UP);
                slideTargetPosition = SLIDE_HALF_OUT;
                currentPresetState = PresetState.IDLE;
                break;
            case FOLD_IN_LOWER:
                slideTargetPosition = SLIDE_COLLAPSED_INTO_ROBOT;
                wristServo.setPosition(WRIST_FOLDED);
                break;
            case COLLECT:
                wristServo.setPosition(WRIST_DOWN);
                currentPresetState = PresetState.IDLE;
                break;
            case COLLECTION_SUCCESSFUL: // TODO: Consider combining with transfer
                slideTargetPosition = SLIDE_COLLAPSED_INTO_ROBOT;
                lowerClawServo.setPosition(INTAKE_OFF);
                currentPresetState = PresetState.IDLE;
                break;
            case PRE_HANG:
                leftActuatorTargetPosition = ACTUATORS_FULLY_EXTENDED;
                currentPresetState = PresetState.IDLE;
                break;
            case HANG:
                leftActuatorTargetPosition = ACTUATORS_HANG;
                currentPresetState = PresetState.IDLE;
                break;
            case IDLE:
                break;
            default:
                telemetry.addData("Error", "Invalid arm state: " + state);
                telemetry.update();
                currentPresetState = PresetState.IDLE; // Set to a safe state
                break;
        }
    }
    /**
     * Controls the gamepad rumble based on the given parameters.
     *
     * @param gamepad      The gamepad to rumble.
     * @param isOverCurrent True if an overcurrent condition is detected, false otherwise.
     */
    private void controlRumble(Gamepad gamepad, boolean isOverCurrent) {
        if (isOverCurrent) {
            // Start rumble if not already rumbling
            if (!isRumbling) {
                gamepad.rumble(1.0, 1.0, 500); // Full intensity, 500ms duration
                rumbleTimer.reset();
                isRumbling = true;
            }
        } else {
            // Stop rumble if overcurrent is no longer detected and it was rumbling
            if (isRumbling) {
                gamepad.stopRumble();
                isRumbling = false;
            }
        }
        if (rumbleTimer.milliseconds() > 500 && isRumbling){
            gamepad.stopRumble();
            isRumbling = false;
        }
    }
}



