package org.firstinspires.ftc.teamcode.in_development;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Disabled
public class TeleopTest extends OpMode {

    // Create a org.firstinspires.ftc.teamcode.in_development.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this, telemetry);
    private Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(-90));
    MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

    double forward;
    double turn;
    double strafe;
    double brake = 0, aLastTime = 0, bLastTime = 0, xLastTime = 0, yLastTime = 0, rBLastTime = 0, lBLastTime = 0, dPadUpLastTime = 0, dpadDownLastTime = 0, rightTriggerLastTime = 0, leftTriggerLastTime = 0;
    boolean aButtonPressed = false, bButtonPressed = false, xButtonPressed = false, yButtonPressed = false, dPadUpPressed = false, dPadDownPressed = false, dPadLeftPressed = false, dPadRightPressed = false,
            rightTriggerPressed = false, leftTriggerPressed = false, backButtonPressed = false, startButtonPressed = false, preHangStarted = false, hangStarted = false;
    final double BUTTON_PRESS_DELAY = .075;// seconds, keep track of how long a button has been pressed and allow for a quick press to move a servo a small amount while a long press moves the servo a longer distance.

    ElapsedTime runtime = new ElapsedTime();

    // This method will be called once, when the INIT button is pressed.
    @Override
    public void init() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play to start OpMode.");
        telemetry.update();
    }

    // This method will be called repeatedly during the period between when the init button is pressed and when the play button is pressed (or theOpMode is stopped).
    @Override
    public void init_loop() {

    }

    // This method will be called once, when the play button is pressed.
    @Override
    public void start() {
        runtime.reset();
    }

    // This method will be called repeatedly during the period between when the play button is pressed and when the OpMode is stopped.
    @Override
    public void loop() {
        robot.opModeActive = true;
        robot.updateArmState(); // update arm state machine to track arm power. By calling it here it gets updated everytime the opMode loops but otherwise works in the background while motors move.
        robot.updateLiftState(); // update lift state machine to track lift power.

        // Run wheels in strafer mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back and left and right, the Right stick turns left and right.
        // This way it's easy to drive diagonally and have good control of heading.

        /// Mr. Morris: Add brake functionality by holding the left trigger varying amounts.
        brake = gamepad1.left_trigger; // 0 to 1
        forward = -gamepad1.left_stick_y * (1 - brake);
        strafe = gamepad1.left_stick_x * (1 -brake);
        turn = gamepad1.right_stick_x * (1 - brake);

        // Combine forward and turn for blended motion. Use org.firstinspires.ftc.teamcode.in_development.RobotHardware class
        drive.setDrivePowers(forward, strafe, turn);
    }

    // This method will be called once, when this OpMode is stopped.
    @Override
    public void stop() {

    }
}
