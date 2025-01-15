package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This OpMode illustrates how to use an external "hardware" class to modularize all the robot's sensors and actuators.
 * This approach is very efficient because the same hardware class can be used by all of your teleop and autonomous OpModes
 * without requiring many copy & paste operations.  Once you have defined and tested the hardware class with one OpMode,
 * it is instantly available to other OpModes.
 *
 * The real benefit of this approach is that as you tweak your robot hardware, you only need to make changes in ONE place (the Hardware Class).
 * So, to be effective you should put as much or your hardware setup and access code as possible in the hardware class.
 * Essentially anything you do with hardware in BOTH Teleop and Auto should likely go in the hardware class.
 *
 * The Hardware Class is created in a separate file, and then an "instance" of this class is created in each OpMode.
 * In order for the class to do typical OpMode things (like send telemetry data) it must be passed a reference to the
 * OpMode object when it's created, so it can access all core OpMode functions.  This is illustrated below.
 *
 * In this concept sample, the hardware class file is called org.firstinspires.ftc.teamcode.RobotHardware.java and it must accompany this sample OpMode.
 * So, if you copy org.firstinspires.ftc.teamcode.CompetitionTeleop.java into TeamCode (using Android Studio or OnBotJava) then org.firstinspires.ftc.teamcode.RobotHardware.java
 * must also be copied to the same location (maintaining its name).
 */

/**
 * Mr. Morris:           TODO: 1) Test, then revise code for arm, lift, gripper, and wrist
 *                              2) Write code and presets for lift. An automated hang sequence is especially needed.
 *                              3) Write code for wrist. - I think this is at least temporarily done, but the wrist servo seems to be in continuous mode so we need to use the SRS programmer, fix that, then test.
 *                              4) Test second lift motor once it is installed
 *                              5) Consider including initial offset for lift so that measurements can be taken from ground
 *                              6) Change code when active intakeLeft and color sensor is implemented
 *                              7) Consider implementing Driver-centric toggle
 *                              8) Set up code for hardware limit switches for arm and lift - Edit: this may not be necessary with the over current sensing
 *
 *     COMPLETE, NEEDS TESTING: 1) Finish updating to match RobotHardware file definitions, then delete or comment out @Disabled
 *                              3) Use math to keep wrist turned so that gripper is level with ground (rotate relative to arm rotation) - Edit: this is no longer necessary and is broken
 *                              4) Add elapsed time tracking and implement better button press delay method. See <a href="https://stemrobotics.cs.pdx.edu/node/7262.html">...</a>
 *                              5) Implemented a state machine method of tracking the arm movement with over current detection to prevent the arm from doing damage to itself. Check it out in the RobotHardware class.
 *                              6) Wrote code for arm presets and incrementation. Needs refinement once we have the lift scoring basket in place
 *                              7) Fixed initial offset code for arm
 *                              8) Wrote code for lift but will need testing and refinement, especially the hang sequence
 */

@TeleOp(name="CompetitionTeleop", group="Teleop")
//@Disabled
public class CompetitionTeleop extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this, telemetry);

    @Override
    public void runOpMode() {
        double forward;
        double turn;
        double strafe;
        double brake = 0, aLastTime = 0, bLastTime = 0, xLastTime = 0, yLastTime = 0, rBLastTime = 0, lBLastTime = 0, dPadUpLastTime = 0, dpadDownLastTime = 0, rightBumperLastTime = 0, leftBumperLastTime = 0;
        boolean aButtonPressed = false, bButtonPressed = false, xButtonPressed = false, yButtonPressed = false, dPadUpPressed = false, dPadDownPressed = false, dPadLeftPressed = false, dPadRightPressed = false,
                rightTriggerPressed = false, leftTriggerPressed = false, backButtonPressed = false, startButtonPressed = false, preHangStarted = false, hangStarted = false;
        final double BUTTON_PRESS_DELAY = .075;// seconds, keep track of how long a button has been pressed and allow for a quick press to move a servo a small amount while a long press moves the servo a longer distance.

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        ElapsedTime runtime = new ElapsedTime();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.opModeActive = true;
            robot.updateArmState(); // update arm state machine to track arm position. By calling it here it gets updated everytime the opMode loops but otherwise works in the background while motors move.
            robot.updateLiftState(); // update lift state machine to track lift position.

            // Run wheels in strafer mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back and left and right, the Right stick turns left and right.
            // This way it's easy to drive diagonally and have good control of heading.

            /// Mr. Morris: Alternatively we could use right trigger for forward, left trigger for reverse, left_stick_x for strafing and right_stick_x for turning,
            ///             then left/right stick_y could be free for arms or something
            brake = Range.clip(gamepad1.right_stick_y, -1, 0);
            forward = -gamepad1.left_stick_y * (1 - brake);
            strafe = gamepad1.left_stick_x * (1 - brake);
            turn = gamepad1.right_stick_x * (1 - brake);

            // Combine forward and turn for blended motion. Use org.firstinspires.ftc.teamcode.RobotHardware class
            robot.mecanumDrive(forward, strafe, turn);

            // Use gamepad left & right Bumpers to open and close the gripper claw
            // Use the SERVO constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            // Each time around the loop, the servos will move by a small amount.
            // Limit the total offset to half of the full travel range
            // Close gripper when right bumper is pressed if it's not already at max, open gripper when left bumper is pressed if it's not already at min
            // Keeps track of how long a button is pressed and moves a small amount for a short press and a larger amount for a long press
            // Use the MOTOR constants defined in org.firstinspires.ftc.teamcode.RobotHardware class.
            if (gamepad1.right_trigger > .1){
                robot.intakeIn(gamepad1.right_trigger);

            } else if (gamepad1.left_trigger > .1){
                robot.intakeOut(gamepad1.left_trigger);
            } else robot.intakeIn(0);

            // Use gamepad X and Y buttons to cycle through arm angle presets
            // Go to next higher arm angle preset
            if (gamepad1.y) {
                if (!yButtonPressed){
                    robot.setArmAngle(robot.getArmNextHigherPreset());
                    yButtonPressed = true;
                }
            } else yButtonPressed = false;
            // Go to next lower lift height preset
            if (gamepad1.x) {
                if (!xButtonPressed){
                    robot.setArmAngle(robot.getArmNextLowerPreset());
                    xButtonPressed = true;
                }
            } else xButtonPressed = false;

            if (gamepad1.a && runtime.seconds() - aLastTime > BUTTON_PRESS_DELAY) {
                robot.armAngleIncrement();
                aLastTime = runtime.seconds();
            }
            if (gamepad1.b && runtime.seconds() - bLastTime > BUTTON_PRESS_DELAY) {
                robot.armAngleDecrement();
                bLastTime = runtime.seconds();
            }


            // Use trigger buttons to rotate wrist up (right trigger) and down (left trigger)
            if (gamepad1.right_bumper && runtime.seconds() - rightBumperLastTime > BUTTON_PRESS_DELAY) {
                robot.wristIncrement();
                rightBumperLastTime = runtime.seconds();
            }
            if (gamepad1.left_bumper && runtime.seconds() - leftBumperLastTime > BUTTON_PRESS_DELAY) {
                robot.wristDecrement();
                leftBumperLastTime = runtime.seconds();
            }

            // Use gamepad Dpad left and right buttons to cycle through lift height presets
            // Go to next higher lift height preset
            if (gamepad1.dpad_right) {
                if (!dPadRightPressed && robot.liftPositionIndex < robot.LIFT_POSITIONS.length - 1) {
                    robot.liftPositionIndex = robot.liftPositionIndex + 1;
                    dPadRightPressed = true;
                }
                robot.setLiftPositionInches(robot.LIFT_POSITIONS[robot.liftPositionIndex]);
            } else dPadRightPressed = false;
            // Go to next lower lift height preset
            if (gamepad1.dpad_left) {
                if (!dPadLeftPressed && robot.liftPositionIndex > 0) {
                    robot.liftPositionIndex = robot.liftPositionIndex - 1;
                    dPadLeftPressed = true;
                }
                robot.setLiftPositionInches(robot.LIFT_POSITIONS[robot.liftPositionIndex]);
            } else dPadLeftPressed = false;

            // Use gamepad Dpad up and down buttons to extend and retract the slides by a preset amount.
            if (gamepad1.dpad_up) {
                if (!dPadUpPressed) {
                    robot.liftIncrementInches(); // extend when Dpad up is pressed
                    dPadUpPressed = true;
                }
            } else dPadUpPressed = false;
            if (gamepad1.dpad_down) {
                if (!dPadDownPressed) {
                    robot.liftDecrementInches(); // retract when Dpad down is pressed
                    dPadDownPressed = true;
                }
            } else dPadDownPressed = false;

            // Cycle through hang off, pre-hang, and hang states. Cycle so that it can be deactivated in case it was started by mistake or something goes wrong.
            if (gamepad1.start) {
                if (!startButtonPressed) {
                    robot.cycleHangState();
                    startButtonPressed = true;
                }
            } else startButtonPressed = false;

            // Toggle Hold Position
            if (gamepad1.back) {
                if (!backButtonPressed) {
                    robot.holdArm = !robot.holdArm;
                    backButtonPressed = true;
                }
            } else backButtonPressed = false;

            /// This code needs updated. We probably don't want the wrist matching the arm angle all the time since it needs to reach back to the lift,
            /// however it might be nice to have it go to preset angles when the arm goes to presets
            // Move wrist so that it moves when arm rotates to keep gripper parallel to floor
            // e.g. if arm angle is at -30 (30 degrees below forward horizontal), wrist must be 30 (30 degrees above forward horizontal) to keep gripper horizontal
//            robot.setWristAngle(-robot.getArmAngle());

            // Send telemetry messages to explain controls and show robot status
            telemetry.addLine("Drive/Strafe: Left Stick");
            telemetry.addLine("Turn: Right Stick");
            telemetry.addLine("Arm Score: Y, Intake: X, Up/Down: B & A Buttons");
            telemetry.addLine("Lift Extend/Retract: Dpad Up & Down Buttons");
            telemetry.addLine("Gripper Open/Closed: Left and Right Bumpers");
            telemetry.addLine("--------");
//            telemetry.addData("Drive Power", "%.2f", forward);
//            telemetry.addData("Strafe Power", "%.2f", strafe);
//            telemetry.addData("Turn Power",  "%.2f", turn);
//            telemetry.addData("Wrist Position", "%.2f", robot.getWristPosition());
//            telemetry.addData("Wrist Angle", "%.2f", robot.getWristAngle());
            telemetry.addData("Arm Position Index", robot.armPositionIndex);
            telemetry.addData("Arm Angle Preset Target", robot.ARM_PRESET_ANGLES[robot.armPositionIndex]);
            telemetry.addData("Arm Angle Relative to Zero", "%.2f",robot.getArmAngleRelativeToZero());
            telemetry.addData("Arm Target Angle", "%.2f",robot.getArmTargetAngle());
            telemetry.addData("Arm Position", "%.2f",robot.getArmEncoderCounts());
            telemetry.addData("Arm Target Position", "%.2f",robot.getArmTargetPosition());
            telemetry.addData("Arm State", robot.getArmState());
            telemetry.addData("Arm Current (Amps)", robot.getArmCurrentAmps());
            telemetry.addData("Lift State", robot.getLiftState());
            telemetry.addData("Lift Position Index", robot.liftPositionIndex);
            telemetry.addData("Lift Preset Target Position", robot.LIFT_POSITIONS[robot.liftPositionIndex]);
            telemetry.addData("Lift Target Position", robot.getLiftTargetPosition());
            telemetry.addData("Left Lift Position", robot.getLeftLiftPosition());
            telemetry.addData("Right Lift Position", robot.getRightLiftPosition());
            telemetry.addData("Left Lift Position Inches", robot.getLeftLiftPositionInches());
            telemetry.addData("Right Lift Position Inches", robot.getRightLiftPositionInches());
            telemetry.addData("Left Lift Current (Amps)",robot.getLiftLeftCurrentAmps());
            telemetry.addData("Right Lift Current (Amps", robot.getLiftRightCurrentAmps());
            telemetry.addData("Runtime", "%.2f",runtime.seconds());
            telemetry.update();

            idle(); //share processor with other programs - good to include in any loop structure in a linear OpMode
        }
        robot.opModeActive = false;
    }
}
