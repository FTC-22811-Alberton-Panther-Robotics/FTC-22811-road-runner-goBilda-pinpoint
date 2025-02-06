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


 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.in_development;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This is (mostly) the OpMode used in the goBILDA Robot in 3 Days for the 24-25 Into The Deep FTC Season.
 * https://youtube.com/playlist?list=PLpytbFEB5mLcWxf6rOHqbmYjDi9BbK00p&si=NyQLwyIkcZvZEirP (playlist of videos)
 * I've gone through and added comments for clarity. But most of the code remains the same.
 * This is very much based on the code for the Starter Kit Robot for the 24-25 season. Those resources can be found here:
 * https://www.gobilda.com/ftc-starter-bot-resource-guide-into-the-deep/
 *
 * There are three main additions to the starter kit bot code, mecanum drive, a linear slide for reaching
 * into the submersible, and a linear slide to hang (which we didn't end up using)
 *
 * the drive system is all 5203-2402-0019 (312 RPM Yellow Jacket Motors) and it is based on a Strafer chassis
 * The arm shoulder takes the design from the starter kit robot. So it uses the same 117rpm motor with an
 * external 5:1 reduction
 *
 * The drivetrain is set up as "field centric" with the internal control hub IMU. This means
 * when you push the stick forward, regardless of robot orientation, the robot drives away from you.
 * We "took inspiration" (copy-pasted) the drive code from this GM0 page
 * (PS GM0 is a world class resource, if you've got 5 mins and nothing to do, read some GM0!)
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html#field-centric
 */

@TeleOp(name="Bozeman-Teleop", group="Competition")
@Disabled
public class TeleopJan27 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor  leftFrontDrive   = null; //the left drivetrain motor
    private DcMotor  rightFrontDrive  = null; //the right drivetrain motor
    private DcMotor  leftBackDrive    = null;
    private DcMotor  rightBackDrive   = null;
    private DcMotor  lift = null; //the lift motor
    private DcMotor  slide = null; // the slide motor
    private Servo    arm = null; //the arm servo
    private Servo    claw = null; //the claw servo on the end of the arm
    private CRServo  intake = null; //the active intake servo on the horizontal slide
    private ElapsedTime sequenceTimer = new ElapsedTime();

    final double LIFT_TICKS_PER_MM = 28 * 12 /120; // RevRobotics 28 ticks/rev motor, with 12:1 gear reduction, and belt travel of 120mm/rev

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_HIGH is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double LIFT_COLLAPSED_INTO_ROBOT = 0;
    final double LIFT_SCORE_SPECIMEN = 100; //TODO: Example value, replace with actual value
    final double SLIDE_COLLAPSED_INTO_ROBOT = 0;
    final double SLIDE_COLLECT = 100; //TODO: Example value, replace with actual value

    /// Mr. Morris: These are the arm states that the gamepad1 dpad buttons will toggle through
    private enum ArmState{SCORE_SPECIMEN, COLLECT, FOLDED, CLEAR_BARRIER, IDLE}
    private ArmState currentArmState = ArmState.FOLDED;

    /* Variables to store the speed the intakeLeft servo should be set at to intakeLeft, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables that are used to set the lift to a specific position */
    double liftPosition = (int) LIFT_COLLAPSED_INTO_ROBOT;
    double slidePosition = (int) SLIDE_COLLAPSED_INTO_ROBOT;

    double cycleTime = 0;
    double loopTime = 0;
    double oldTime = 0;

    @Override
    public void runOpMode() {
        /// Mr. Morris: Here is code for the time tracker. This gets reset once the driver presses
        ///             play and then is used to set other time tracking variables.
        ElapsedTime runtime = new ElapsedTime();


        //These variables are private to the OpMode, and are used to control the drivetrain.
        double left;
        double right;
        double forward;
        double rotate;
        double max;

        /* Define and Initialize Motors */
        leftFrontDrive  = hardwareMap.dcMotor.get("left_front_drive");
        leftBackDrive   = hardwareMap.dcMotor.get("left_rear_drive");
        rightFrontDrive = hardwareMap.dcMotor.get("right_front_drive");
        rightBackDrive  = hardwareMap.dcMotor.get("right_rear_drive");
        lift   = hardwareMap.get(DcMotor.class, "lift"); //the lift motor
        slide  = hardwareMap.get(DcMotor.class, "slide"); //the slide motor
        arm    = hardwareMap.get(Servo.class, "arm"); //the arm servo
        claw   = hardwareMap.get(Servo.class, "claw"); //the claw servo on the end of the arm
        intake = hardwareMap.get(CRServo.class, "intake"); //the active intake servo on the horizontal slide)

        /* we need to reverse the left side of the drivetrain so it doesn't turn when we ask all the
        drive motors to go forward. */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) lift).setCurrentAlert(5,CurrentUnit.AMPS);
        ((DcMotorEx) slide).setCurrentAlert(5,CurrentUnit.AMPS);

        /* Before starting the lift motor, we'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and Initialize Servos */
        arm = hardwareMap.get(Servo.class, "arm");
        intake = hardwareMap.get(CRServo.class, "intake");

        /* Make sure that the intakeLeft is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot( //TODO: Make sure this orientation is correct!
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {
            // This calls the method at the bottom of the file which has a switch statement to move
            // the arm, slide, and wrist together in coordinated movements.
            updateArmState(currentArmState);

            // These call the method squareInputWithSign which has the effect of dampening small movements
            // to makes it easier to make small precise moves with the joysticks
            double y = -squareInputWithSign(gamepad1.left_stick_y); // left stick is negative when up so flip that with negative sign
            double x = squareInputWithSign(gamepad1.left_stick_x);
            double rx = squareInputWithSign(gamepad1.right_stick_x);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            /* TECH TIP: If Else statement:
            We're using an else if statement on "gamepad1.x" and "gamepad1.b" just in case
            multiple buttons are pressed at the same time. If the driver presses both "a" and "x"
            at the same time. "a" will win over and the intakeLeft will turn on. If we just had
            three if statements, then it will set the intakeLeft servo's power to multiple speeds in
            one cycle. Which can cause strange behavior. */
            // TODO: Use a color sensor to automatically reject wrong color samples and stop the intake when a sample is loaded
            if (gamepad1.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad1.right_bumper) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else if (gamepad2.left_bumper) {
                intake.setPower(INTAKE_COLLECT);
            }
            else if (gamepad2.right_bumper) {
                intake.setPower(INTAKE_DEPOSIT);
            }
            else {
                intake.setPower(INTAKE_OFF);
            }
            // 2500 rpm is the speed we would like the lift to run at, adjust as needed. Each loop through the program
            // this will actually only travel a small fraction of this amount but this is a way to smooth it out across loops
            liftPosition -= gamepad2.left_stick_y * 2500 * cycleTime;

            /* Here we implement a set of if else statements to set our arm to different scoring positions.
            We check to see if a specific button is pressed, and then move the arm (and sometimes
            intake and wrist) to match. For example, if we click the right bumper we want the robot
            to start collecting. So it moves the liftPosition to the ARM_COLLECT position,
            it folds out the wrist to make sure it is in the correct orientation to intakeLeft, and it
            turns the intakeLeft on to the COLLECT mode.*/

            if (gamepad1.dpad_left) {
                /* This raises the arm up just enough to clear the barrier to collect samples in the submersible */
                currentArmState = ArmState.CLEAR_BARRIER;
            }

            else if (gamepad1.dpad_right){
                /* This lowers the arm to the ground and turns on the intakeLeft wheels */
                sequenceTimer.reset();
                currentArmState = ArmState.COLLECT;
            }

            else if (gamepad1.dpad_up){
                /* This raises and extends the arm up to score a specimen on the high rung */
                currentArmState = ArmState.SCORE_SPECIMEN;
            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down, brings the slide in, and folds the wrist in */
                updateArmState(ArmState.FOLDED);
            }

            /* Here we set the target position of our life to match the variable that was selected
            by the driver. We add the liftPosition Variable to our armPositionFudgeFactor, before adding
            our armSlideComp, which adjusts the arm height for different lift extensions.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            lift.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) lift).setVelocity(2100);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Check to see if our lift is over the current limit, and report via telemetry. */
            if (((DcMotorEx) lift).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }

            /* This is how we check our loop time. We create three variables:
                loopTime is the current time when we hit this part of the code.
                cycleTime is the amount of time in seconds our current loop took.
                oldTime is the time in seconds that the previous loop started at.
            We find cycleTime by just subtracting the old time from the current time. For example,
            lets say it is 12:01.1, and then a loop goes by and it's 12:01.2. We can take the current
            time (12:01.2) and subtract the oldTime (12:01.1) and we're left with just the difference,
            0.1 seconds. */
            loopTime = getRuntime();
            cycleTime = loopTime - oldTime;
            oldTime = loopTime;

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("lift Target Position: ", lift.getTargetPosition());
            telemetry.addData("lift Encoder: ", lift.getCurrentPosition());
            telemetry.addData("lift mm: ", lift.getCurrentPosition()/LIFT_TICKS_PER_MM);
            telemetry.update();

            // We may want to try the following gemini-suggested improvement to the telemetry (fewer telemetry calls improves performance)
//            telemetry.addData("Arm Info", "Target: %d, Encoder: %d, Angle: %.2f",
//                    armMotor.getTargetPosition(), armMotor.getCurrentPosition(),
//                    armMotor.getCurrentPosition() / ARM_TICKS_PER_DEGREE);
//            telemetry.update();
        }
    }

    /*  Mr. Morris - I got this idea from https://github.com/alan412/LearnJavaForFTC/blob/master/LearnJavaForFTC.pdf
     *  This function squares input but keeps the sign. This is used to alter joystick input to be less sensitive at small values.
     *  This works because small values less than one become even smaller when squared but values near one have little change. In
     *  this way, it is easier to have fine control over the robot to make small movements.
     */
    private double squareInputWithSign(double input) {
        if (input < 0) {
            return -input * input;
        } else {
            return input * input;
        }
    }

    /**
     * Mr. Morris:
     * This method is a finite state machine implementation to track the possible positions of the arm, slide, and wrist. The
     * main reason this was implemented is because Mahlon asked for a time delay when moving from the SCORE_SPECIMEN down. I
     * interpreted this as hitting the button to go to the COLLECT, however it could be adjusted to behave how the drivers want.
     * Currently it has a timer in the COLLECT case so that the arm delays half a second while the slide retracts. There are
     * certainly simpler solutions to this problem, but this one is more robust and gives us the framework to make all sorts of
     * fancy transitions between states while still being readable. The magic happens at the top of the while(opModeIsActive())
     * loop. It calls this method to update the arm state every loop. Later in the program buttons will change the currentState
     * and the next program loop when the method is called it will update the positions. By being called every program loop
     * regardless of button press it can track timers and even flow from one arm state to another if we want it to.
     * @param state This is an enum of ArmState type. enums are a way of creating a sort of list of options that can be chosen
     *              by name in a switch statement like this. This makes code more readable and less prone to errors. It is a
     *              description of a position the arm, slide, and wrist can be in (e.g. FOLDED, COLLECT, etc.)
     */
    private void updateArmState(ArmState state){
        switch (state){
            case SCORE_SPECIMEN:
                liftPosition = LIFT_SCORE_SPECIMEN;
                currentArmState = ArmState.IDLE;
                break;
            case COLLECT:
                slidePosition = SLIDE_COLLECT;
                currentArmState = ArmState.IDLE;
                break;
            case FOLDED:
                intake.setPower(INTAKE_OFF);
                currentArmState = ArmState.IDLE;
                break;
            case IDLE:
                break;
            default:
                telemetry.addData("Error", "Invalid arm state: " + state);
                telemetry.update();
                throw new IllegalArgumentException("Invalid arm state: " + state);
        }
    }
}