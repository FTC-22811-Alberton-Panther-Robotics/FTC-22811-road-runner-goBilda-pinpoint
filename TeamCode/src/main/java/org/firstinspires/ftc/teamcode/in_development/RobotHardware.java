package org.firstinspires.ftc.teamcode.in_development;
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This file works in conjunction with the External Hardware Class sample called: org.firstinspires.ftc.teamcode.in_development.CompetitionTeleop.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes six motors (left_front_drive, left_rear_drive, right_front_drive, right_rear_drive, arm_rotate, and arm_extend) and two servos (wrist and gripper)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named org.firstinspires.ftc.teamcode.in_development.RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample org.firstinspires.ftc.teamcode.in_development.CompetitionTeleop.java, and select TeleOp.
 *
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    private Telemetry telemetry;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(OpMode opmode, Telemetry telemetry) {
        this.telemetry = telemetry;
        myOpMode = opmode;
    }
    // This is how the opMode tells the RobotHardware class when it is active, if there are any loops in the hardware
    // class they should check this to avoid getting stuck in a loop when the stop button is hit
    public boolean opModeActive = false;

    // Define motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor arm = null;
    private DcMotor leftLift = null;
    private DcMotor rightLift = null;
    private DcMotorEx armEx = null;
    private DcMotorEx leftLiftEx = null;
    private DcMotorEx rightLiftEx = null;
    private Servo   wrist = null;
    private Servo   gripper = null;

    // Gripper and Wrist Constants
    private static final double GRIPPER_INCREMENT = 0.06, GRIPPER_MAX = 1, GRIPPER_MIN = 0 ;  // sets rate to move gripper servo and max and min travel. If you use SRS servo programmer to set limits, this will be 1 and 0. If you need to limit travel in the software, this is where to do it.
    private static final double WRIST_INCREMENT = 0.02 ; // sets rate to move wrist servo
    private static final double WRIST_MAX_ANGLE  = 300 ; // Adjust this angle if SRS servo programmer has limited servo travel to less than 300

    // Arm Constants
    private static final long ARM_POSITION_TIMEOUT = 3000;
    private static final int ARM_INCREMENT_DEGREES = 5, ARM_ROTATE_MAX = 160, ARM_ROTATE_MIN = -20, ARM_ROTATE_ENCODER_RESOLUTION = 28, ARM_ROTATE_GEAR_RATIO = 60, // Straight forward defined as 0 degrees
            ARM_STARTING_ANGLE_OFFSET = 120, ARM_STOW = ARM_STARTING_ANGLE_OFFSET, ARM_INTAKE = 0, SPECIMEN_INTAKE = 30, CHAMBER_SCORE = 70, ARM_OUTTAKE = 100;
    public final double[] ARM_ANGLES = {ARM_STOW, ARM_OUTTAKE, CHAMBER_SCORE, SPECIMEN_INTAKE, ARM_INTAKE, ARM_ROTATE_MIN};
    private static final double ARM_MAX_CURRENT_AMPS = 7;
    // Create state machines to track what state the arm and lift motors are in. A state machine is a computational model that represents a system
    // with a finite number of states and transitions between those states. It's a powerful tool for managing complex logic and behavior,
    // especially in systems with dynamic or event-driven interactions. Check out the updateArmState and updateLiftState functions to see how this works.
    public enum ArmState {IDLE, MOVING_TO_TARGET, HOLDING_POSITION, STALLED, TIMEOUT, STOW, ERROR}
    private ArmState armCurrentState = ArmState.IDLE;
    ElapsedTime armStateTimer = new ElapsedTime();
    // Arm Variables
    public int armPositionIndex = 0;
    private int armTargetPosition = 0;
    public boolean holdArm = true; // consider triggering the hold behavior in the opMode with a button press. TRUE: maintain arm power after RUN_TO_POSITION is complete, FALSE: stop arm power

    // Lift Constants
    private static final long LIFT_POSITION_TIMEOUT = 3000;
    private static final int LIFT_ENCODER_RESOLUTION = 28, LIFT_GEAR_RATIO = 20;
    private static final double LIFT_TRAVEL_PER_ROTATION_INCHES = 120 / 25.4, LIFT_INCREMENT_INCHES = 1;
    private static final double LIFT_FULLY_RETRACTED = 17.5, LOW_TO_HIGH_RUNG = 16 + LIFT_FULLY_RETRACTED, LOW_RUNG = 20, LOW_BASKET = 25.75, HIGH_BASKET = 43, LIFT_3_STAGE_EXTENDED = 28.8 + LIFT_FULLY_RETRACTED;
    public final double[] LIFT_POSITIONS = {LIFT_FULLY_RETRACTED, LOW_RUNG, LOW_BASKET, HIGH_BASKET, LIFT_3_STAGE_EXTENDED};
    private static final double LIFT_MAX_CURRENT_AMPS = 4;

    // Lift Variables
    public int liftPositionIndex = 0;
    private int liftTargetPosition = 0;
    private int leftLiftDistanceToTarget =0;
    private int rightLiftDistanceToTarget =0;
    private enum LiftState {IDLE, MOVING_UP, MOVING_DOWN, HOLDING_POSITION, STALLED, TIMEOUT, ERROR}
    public LiftState liftCurrentState = LiftState.IDLE;
    private enum HangState {RESET, PRE_HANG, HANG_ONE, HANG_TWO, HANG_THREE, HANG_COMPLETE, NONE};
    private HangState hangCurrentState = HangState.NONE;
    private int hangIndex = 0;
    ElapsedTime hangStateTimer = new ElapsedTime();

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        arm = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "left_lift");
        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "right_lift");

        // Take advantage of the extended features of the DcMotorEx class, such as reading current draw and setting velocity, if the motor supports it
        if (arm instanceof DcMotorEx) {
            armEx = (DcMotorEx) arm;
        } else telemetry.addData("WARNING: ", "Arm is not a DcMotorEx");
        if (leftLift instanceof DcMotorEx) {
            leftLiftEx = (DcMotorEx) leftLift;
        } else telemetry.addData("WARNING: ", "Left lift is not a DcMotorEx");
        if (rightLift instanceof DcMotorEx) {
            rightLiftEx = (DcMotorEx) rightLift;
        } else telemetry.addData("WARNING: ", "Right lift is not a DcMotorEx");

        // Set the directions of the lift motors
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders at start of program
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Since there are encoders connected, RUN_USING_ENCODER mode is enabled for greater accuracy
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        wrist = myOpMode.hardwareMap.get(Servo.class, "wrist");
        gripper = myOpMode.hardwareMap.get(Servo.class, "gripper");

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /** Arm Code
    * The following function creates a state machine. A state machine is a computational model that represents a system with a finite number of states and transitions between
    * those states. It's a powerful tool for managing complex logic and behavior, especially in systems with dynamic or event-driven interactions.
    * Key Components:
    * States: Represent the different possible conditions or modes of the system (e.g., "idle," "moving," "holding," "error").
    * Transitions: Define the rules for moving from one state to another based on events or conditions (e.g., sensor readings, user input, timeouts).
    * Actions: Specify the actions to be performed when entering or exiting a state or during a transition (e.g., setting motor power, updating variables, displaying messages)
    */
     public void updateArmState(){
        switch (armCurrentState) {
            // In the idle state it shuts off power to the motor as long as it stays close to its target power,
            // this saves battery but is more prone to drooping and wobble from engaging & disengaging the motor
            case IDLE:
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (Math.abs(armTargetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                    armStateTimer.reset();
                }
                break;
            case MOVING_TO_TARGET:
                arm.setTargetPosition(armTargetPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int distance = armTargetPosition - arm.getCurrentPosition(); // This is used to calculate which direction the motor is going to move to adjust gravity correctly (add speed when going up, slow when going down)
                double gravityCompensation = 300 * Math.cos(Math.toRadians(calculateAngleFromEncoderValue(arm.getCurrentPosition()))); // Adjust gravity compensation as needed
                double velocity = 1000; // TODO: See https://www.ctrlaltftc.com/feedforward-control. This should not be setting velocity. The intention here is to create a Feed Forward effect that compensates the arm movement for gravity, however power should be set instead, or perhaps acceleration.
                if (distance >= 0) {velocity += gravityCompensation;}
                else {velocity -= gravityCompensation;}
                armEx.setVelocity(velocity);
                if (armStateTimer.seconds() > ARM_POSITION_TIMEOUT) {
                    armCurrentState = ArmState.TIMEOUT;
                    armStateTimer.reset();
                }
                else if (armEx.getCurrent(CurrentUnit.AMPS) > ARM_MAX_CURRENT_AMPS){
                    armCurrentState = ArmState.STALLED;
                    armStateTimer.reset();
                }
                else if (!arm.isBusy()) {
                    if (holdArm) armCurrentState = ArmState.HOLDING_POSITION;
                    else armCurrentState = ArmState.IDLE;
                }
                break;
            // The holding power state only happens if holdArm is toggled true (Press the back button in CompetitionTeleop)
            // it will keep power on to the motor the whole time, which helps maintain power more rigidly but draws more power.
            case HOLDING_POSITION:
                if (!holdArm) {armCurrentState = ArmState.IDLE;}
                else if (Math.abs(armTargetPosition - arm.getCurrentPosition()) > armEx.getTargetPositionTolerance()) {
                    armCurrentState = ArmState.MOVING_TO_TARGET;
                }
                break;
            case STALLED:
                armTargetPosition = arm.getCurrentPosition();
                arm.setPower(0);
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (armStateTimer.seconds() > 1 && armEx.getCurrent(CurrentUnit.AMPS) < ARM_MAX_CURRENT_AMPS) armCurrentState = ArmState.IDLE;
                break;
            case TIMEOUT:
                armTargetPosition = arm.getCurrentPosition();
                if(armStateTimer.seconds() > 1){
                    armCurrentState = ArmState.IDLE;
                    armStateTimer.reset();
                }
            case STOW:
                armTargetPosition = ARM_STOW;
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            // There isn't currently any way the error state triggers. We may be able to get rid of this, but it would be good to get more error handling in the code, so I'm leaving it for now as a reminder
            case ERROR:
                telemetry.addData("ERROR:", " arm state unable to be read");
                telemetry.update();
                break;
        }
    }
    // Public method that Opmodes can call to set the target power for the arm state machine
    public void setArmAngle(double targetAngle){
        this.armTargetPosition = (int) calculateArmEncoderValue(targetAngle - ARM_STARTING_ANGLE_OFFSET);
    }
    // Public method that Opmodes can call to get what state the arm is in
    public ArmState getArmState(){
        return armCurrentState;
    }
    // Public method that Opmodes can call if needed to reset the arm state
    public void resetArmState(){
        armCurrentState = ArmState.IDLE;
    }
    public double getArmCurrentAmps(){
        return armEx.getCurrent(CurrentUnit.AMPS);
    }
    // Function to calculate encoder power from target angle
    private int calculateArmEncoderValue(double armAngle) {
        int ticksPerRevolution = ARM_ROTATE_ENCODER_RESOLUTION * ARM_ROTATE_GEAR_RATIO;
        double ticksPerDegree = (double) ticksPerRevolution / 360;
        return (int) (armAngle * ticksPerDegree);
    }
    // Function to calculate arm angle from encoder power
    private double calculateAngleFromEncoderValue(int encoderPosition) {
        int ticksPerRevolution = ARM_ROTATE_ENCODER_RESOLUTION * ARM_ROTATE_GEAR_RATIO;
        double degreesPerTick = 360.0 / ticksPerRevolution;
        return encoderPosition * degreesPerTick;
    }
    public void armAngleIncrement(){
        if (getArmAngleRelativeToZero() < ARM_ROTATE_MAX) {
            setArmAngle(getArmAngleRelativeToZero() + ARM_INCREMENT_DEGREES);
        }
    }
    public void armAngleDecrement(){
        if (getArmAngleRelativeToZero() > ARM_ROTATE_MIN){
            setArmAngle(getArmAngleRelativeToZero() - ARM_INCREMENT_DEGREES);
        }
    }
    // Get the encoder information for the arm rotation motor and convert it to degrees.Adjust for starting/resting power.
    public double getArmAngleRelativeToZero(){
        int encoderCounts = arm.getCurrentPosition();
        double angleRelativeToZero = calculateAngleFromEncoderValue(encoderCounts) + ARM_STARTING_ANGLE_OFFSET;
        return angleRelativeToZero;
    }
    public double getArmEncoderCounts(){
        return arm.getCurrentPosition();
    }
    public double getArmTargetAngle(){
        return calculateAngleFromEncoderValue(armTargetPosition);
    }
    public double getArmTargetPosition(){
        return armTargetPosition;
    }

    // Lift Code
    public void updateLiftState(){
        switch (liftCurrentState) {
            case IDLE:
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // If either motor is far enough from the target distance then go into moving up state or moving down state as needed.
                if (Math.abs(leftLiftDistanceToTarget) > leftLiftEx.getTargetPositionTolerance() || Math.abs(rightLiftDistanceToTarget) > rightLiftEx.getTargetPositionTolerance()) {
                    if ((leftLiftDistanceToTarget + rightLiftDistanceToTarget)/2 >= 0) liftCurrentState = LiftState.MOVING_UP;
                    else if ((leftLiftDistanceToTarget + rightLiftDistanceToTarget)/2 < 0) liftCurrentState = LiftState.MOVING_DOWN;
                    hangStateTimer.reset();
                }
                break;
            // The moving up state is the same as the moving down state, except the moving down state resets its lower limit when it stalls
            case MOVING_UP:
                leftLift.setTargetPosition(liftTargetPosition);
                rightLift.setTargetPosition(liftTargetPosition);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftEx.setVelocity(1000);
                rightLiftEx.setVelocity(1000);
                if (hangStateTimer.seconds() > LIFT_POSITION_TIMEOUT) {
                    liftCurrentState = LiftState.TIMEOUT;
                    hangStateTimer.reset();
                }
                else if (leftLiftEx.getCurrent(CurrentUnit.AMPS) > ARM_MAX_CURRENT_AMPS){
                    liftCurrentState = LiftState.STALLED;
                    hangStateTimer.reset();
                }
                else if (!leftLift.isBusy()) {
                    liftCurrentState = LiftState.IDLE;
                }
                break;
            // The moving down state is different from the hang routine because it is not expecting much resistance on the motors until it hits
            // the max retraction lower limit, so it uses the stall condition to reset the encoder
            case MOVING_DOWN:
                leftLift.setTargetPosition(liftTargetPosition);
                rightLift.setTargetPosition(liftTargetPosition);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (hangStateTimer.seconds() > LIFT_POSITION_TIMEOUT) {
                    liftCurrentState = LiftState.TIMEOUT;
                    hangStateTimer.reset();
                }
                else if (leftLiftEx.getCurrent(CurrentUnit.AMPS) > LIFT_MAX_CURRENT_AMPS){
                    liftCurrentState = LiftState.STALLED;
                    leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    hangStateTimer.reset();
                }
                else if (!leftLift.isBusy() && !rightLift.isBusy()) {
                    liftCurrentState = LiftState.IDLE;
                }
                break;
            case HOLDING_POSITION: // Not currently used
                break;
            case STALLED:
                liftTargetPosition = leftLift.getCurrentPosition(); // pick the left lift, either will do - hopefully they're pretty much the same anyway but the other states will take care of that if they are.
                leftLift.setPower(0);
                rightLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (hangStateTimer.seconds() > 1 && leftLiftEx.getCurrent(CurrentUnit.AMPS) < LIFT_MAX_CURRENT_AMPS
                        && rightLiftEx.getCurrent(CurrentUnit.AMPS) < LIFT_MAX_CURRENT_AMPS) liftCurrentState = LiftState.IDLE;
                break;
            case TIMEOUT:
                liftTargetPosition = leftLift.getCurrentPosition(); // pick the left lift, either will do - hopefully they're pretty much the same anyway but the other states will take care of that if they are.
                if(hangStateTimer.seconds() > 1) {
                    liftCurrentState = LiftState.IDLE;
                    hangStateTimer.reset();
                }
                break;
            // There isn't currently any way the error state triggers. We may be able to get rid of this, but it would be good to get more error handling in the code, so I'm leaving it for now as a reminder
            case ERROR:
                telemetry.addData("ERROR", " lift state unable to be read");
                telemetry.update();
                break;
        }
    }
    // Public method that Opmodes can call to set the target power for the lift state machine
    public void setLiftPosition(int targetPosition){
        this.liftTargetPosition = targetPosition;
        this.leftLiftDistanceToTarget = targetPosition - leftLift.getCurrentPosition();
        this.rightLiftDistanceToTarget = targetPosition - rightLift.getCurrentPosition();
    }
    public int getLeftLiftPosition(){
        return leftLift.getCurrentPosition();
    }
    public int getLeftLiftPositionInches(){
        return calculateLiftInchesFromEncoderPosition(leftLift.getCurrentPosition());
    }
    public int getRightLiftPosition(){
        return rightLift.getCurrentPosition();
    }
    public int getRightLiftPositionInches(){
        return calculateLiftInchesFromEncoderPosition(rightLift.getCurrentPosition());
    }
    public int getLiftTargetPosition(){
        return liftTargetPosition;
    }
    public void setLiftPositionInches(double targetPositionInches) {
        setLiftPosition(calculateEncoderPositionFromLiftInches(targetPositionInches));
    }
    public void liftIncrementInches(){
        setLiftPositionInches(getLeftLiftPositionInches() + LIFT_INCREMENT_INCHES);
    }
    public void liftDecrementInches(){
        setLiftPositionInches(getLeftLiftPositionInches() - LIFT_INCREMENT_INCHES);
    }
    private int calculateLiftInchesFromEncoderPosition(int encoderPosition) {
        int ticksPerRevolution = LIFT_ENCODER_RESOLUTION * LIFT_GEAR_RATIO;
        double inchesPerTick = LIFT_TRAVEL_PER_ROTATION_INCHES / ticksPerRevolution;
        return (int) ((encoderPosition * inchesPerTick) + LIFT_FULLY_RETRACTED);
    }
    private int calculateEncoderPositionFromLiftInches(double liftInches) {
        int ticksPerRevolution = LIFT_ENCODER_RESOLUTION * LIFT_GEAR_RATIO;
        double inchesPerTick = LIFT_TRAVEL_PER_ROTATION_INCHES / ticksPerRevolution;
        return (int) ((liftInches - LIFT_FULLY_RETRACTED) / inchesPerTick);
    }
    // Public method that Opmodes can call to get what state the lift is in
    public LiftState getLiftState(){
        return liftCurrentState;
    }
    public double getLiftLeftCurrentAmps(){
        return leftLiftEx.getCurrent(CurrentUnit.AMPS);
    }
    public double getLiftRightCurrentAmps(){
        return rightLiftEx.getCurrent(CurrentUnit.AMPS);
    }
    public void resetLiftState(){
        liftCurrentState = LiftState.IDLE;
    }

    // Hang Routine Code
    public void updateHangState(){
        switch (hangCurrentState) {
            case RESET:
                resetArmState();
                resetLiftState();
                hangCurrentState = HangState.NONE;
                break;
            // need to finish hang routine - this won't look for a stall condition, or at least it will have a higher max current allowed as it's expected that the motors will have to work hard to lift the whole robot.
            case PRE_HANG:
                leftLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_RUNG));
                rightLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_RUNG));
                armCurrentState = ArmState.STOW;
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangStateTimer.reset();
                break;
            case HANG_ONE:
                leftLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                rightLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (hangStateTimer.seconds() > 4) { // tune the timeout with testing - less is better - this is to prevent a situation where the motors are basically in place but jittering
                    hangCurrentState = HangState.HANG_TWO;
                    hangStateTimer.reset();
                }
                else if (!leftLift.isBusy() && !rightLift.isBusy()){
                    hangCurrentState = HangState.HANG_TWO;
                    hangStateTimer.reset();
                }
                break;
            case HANG_TWO:
                leftLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_TO_HIGH_RUNG));
                rightLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LOW_TO_HIGH_RUNG));
                if (hangStateTimer.seconds() > 4) { // tune the timeout with testing - less is better - this is to prevent a situation where the motors are basically in place but jittering
                    hangCurrentState = HangState.HANG_THREE;
                    hangStateTimer.reset();
                }
                else if (!leftLift.isBusy() && !rightLift.isBusy()){
                    hangCurrentState = HangState.HANG_THREE;
                    hangStateTimer.reset();
                }
                break;
            case HANG_THREE:
                leftLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                rightLift.setTargetPosition(calculateEncoderPositionFromLiftInches(LIFT_FULLY_RETRACTED));
                if (hangStateTimer.seconds() > 4) { // tune the timeout with testing - less is better - this is to prevent a situation where the motors are basically in place but jittering
                    hangCurrentState = HangState.HANG_COMPLETE;
                    hangStateTimer.reset();
                }
                else if (!leftLift.isBusy() && !rightLift.isBusy()){
                    hangCurrentState = HangState.HANG_COMPLETE;
                    hangStateTimer.reset();
                }
                break;
            case HANG_COMPLETE:
                break;
            case NONE:
                hangStateTimer.reset();
                break;
        }
    }
    // Public method that Opmodes can call to execute the pre-hang routine
    public void cycleHangState(){
        hangIndex +=1;
        hangIndex = hangIndex % 2; // Only cycle between reset, pre-hang, and hang-one
        hangCurrentState = HangState.values()[hangIndex];
    }

    // Gripper Code
    /**
     * Send the gripper the new power to go to
     * @param position value from 0 to 1
     */
    public void setGripperPosition(double position) {
        gripper.setPosition(position);
    }
    public void gripperIncrement(){
        if (gripper.getPosition() + GRIPPER_INCREMENT <= 1) {
            setGripperPosition(gripper.getPosition() + GRIPPER_INCREMENT);
        }
    }
    public void gripperDecrement(){
        if (gripper.getPosition() - GRIPPER_INCREMENT >= 0) {
            setGripperPosition((gripper.getPosition() - GRIPPER_INCREMENT));
        }
    }
    public double getGripperPosition(){
        return gripper.getPosition();
    }

    // Wrist Code
    /**
     * Send the wrist to a certain power
     * @param position is a number from 0 to 1
     */
    public void setWristPosition(double position){
       wrist.setPosition(position);
    }
    public void setWristAngle(double angle){
        setWristPosition(RangeUtils.scale(angle, 0, WRIST_MAX_ANGLE, 0, 1));
    }
    public double getWristPosition(){
        return wrist.getPosition();
    }
    public void wristIncrement() {
        setWristPosition(Math.min(1, wrist.getPosition() + WRIST_INCREMENT));
    }
    public void wristDecrement() {
        if (wrist.getPosition() > 0) {
            setWristPosition(Math.max(0, wrist.getPosition() - WRIST_INCREMENT));
        }
    }
    public double getWristAngle(){
        return RangeUtils.scale(wrist.getPosition(), 0,1, 0, WRIST_MAX_ANGLE);
    }

    // General Functions
    // Class to scale and clip values - this functionality is built into opModes but I created my own here for use in the RobotHardware class
    public static class RangeUtils {
        public static double scale(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
            return outputMin + (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
        }
        public static double clip( double value, double min, double max) {
            if (value < min) return min;
            else if (value > max) return max;
            else return value;
        }
    }
}