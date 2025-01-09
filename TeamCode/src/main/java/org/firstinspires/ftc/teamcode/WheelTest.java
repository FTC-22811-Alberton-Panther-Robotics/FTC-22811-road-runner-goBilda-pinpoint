package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//TODO: Comment out the @Disabled line (add a couple of slashes // before it) when you are ready for this program to be visible on the driver hub
@Disabled
@TeleOp(name="Wheel Test Code")
public class WheelTest extends LinearOpMode {
    public DcMotor leftFrontDrive;

    @Override
    public void runOpMode() {
        /*
         * declare the name of each motor and have it assign the motor by looking in the hardware map
         * (aka the config file) for the name that matches the string in quotes
         * TODO: declare the other 3 motors similar to this one
         */
        leftFrontDrive = hardwareMap.dcMotor.get("left_front_drive");

        /*
         * we need to reverse the left side of the drivetrain because of the way the motors face.
         * TODO: set the leftRearDrive to REVERSE as well
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        // Everything above this happens when the initialize button is pressed. Everything below happens when the start button is pressed.
        waitForStart();

        /*
         * Any while or other loops need to check to make sure the opMode is active. That is, it should exit the loop and exit the program
         * if the stop button has been pressed on the driver hub. This while loop is where any repeated actions should be placed.
         * If you have fairly simple code without any sleep commands in it (sleep commands are bad practice as they are blocking
         * - they prevent other things from happening during that time), it will cycle through the loop roughly 20 times per second depending
         * on processor load
         */
        while (opModeIsActive()) {

            // This checks if the x button is pressed and sets the left front wheel power to 30%
            if (gamepad1.x){
                leftFrontDrive.setPower(.3);
            } else leftFrontDrive.setPower(0);
            // TODO: do the same for the other motors (leftRearDrive, rightFrontDrive, and rightRearDrive) with the y, a, and b buttons

        }
    }
}
