package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;

/*
 * This is a program I (Mr. Morris) did some research on and then wrote to increment through and power motors one at a time on each Control Hub and
 * Expansion Hub port. Sometimes when a robot has the wiring all bundled up it is hard to trace wires and know for sure what is causing motors to
 * behave erratically or troubleshoot configuration issues. This program aims to solve that. It doesn't need to know the configured name of the motor
 * on each port. It just tries the port number and powers it. This can help identify, for instance that port 3 on the Expansion Hub is connected to
 * the lift arm motor and port 0 is the right front drive motor, etc. This will only work if the config file tells the robot that a motor is plugged
 * into the port, but the names don't necessarily have to be correct and indeed this would be a way of telling if they are incorrectly mapped. Use
 * the dPad left and right buttons to cycle through the motor ports on both Control Hub and Driver Hub, and the left stick y axis to power the motor
 * forward (up) or reverse (down).
 */
@TeleOp(name = "MotorPortTest", group = "Test")
public class MotorPortTest extends LinearOpMode {
    private int motorPortNumber = 0; // Change this to the port number of your motor
    private int selectionIndex = 0;
    private DcMotorControllerEx motorController;
    String[] hubOptions = {"Control Hub", "Expansion Hub 2"};
    String selectedHub = null;
    private boolean dpadRightPressed = false;
    private boolean dpadLeftPressed = false;
    private double power = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            // Access the motor by its port number
            boolean exceptionCaught = false;
            selectionIndex = selectionIndex % 8; // There are 8 motor ports, 4 on Control Hub, 4 on Expansion Hub - cycle selection back to start when it reaches the end
            motorPortNumber = selectionIndex % 4; // There are 4 motor ports on each hub - cycle back to start when it reaches the end
            if (selectionIndex / 4 == 0) selectedHub = hubOptions[0]; // The first four ports are on the Control Hub (0), the second four are on the Expansion Hub (1)
            else selectedHub = hubOptions[1];
            try {
                motorController = hardwareMap.get(DcMotorControllerEx.class, selectedHub);
                motorController.setMotorMode(motorPortNumber, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } catch (IllegalArgumentException e) {
                telemetry.addData("Error", "motor not found on" + selectedHub + " port " + motorPortNumber);
                exceptionCaught = true;
            }

            if (!exceptionCaught) {
                // Control the motor with gamepad input
                power = gamepad1.left_stick_y;
                motorController.setMotorPower(motorPortNumber, power);
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                selectionIndex++;
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                selectionIndex--;
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }
            // Telemetry
            telemetry.addData("Selected Hub", selectedHub);
            telemetry.addData("motor Power", power);
            telemetry.addData("Selection Index", selectionIndex);
            telemetry.addData("motor Port", motorPortNumber);
            telemetry.update();
        }
    }
}