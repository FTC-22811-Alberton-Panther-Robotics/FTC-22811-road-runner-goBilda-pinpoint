package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Created a new HardwareClasses file that follows the example "Building a CENTERSTAGE Autonomous"
 * on the RoadRunner guide website: https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
 * I only brought in the parts about creating classes for each hardware component that an autonomous
 * roadrunner opMode would then implement. This is similar to the RobotHardware concept file we used
 * before but using more of a class/object structure to organize the code. It will need to be adapted
 * to take specific target positions, have incrementing functions and presets, etc. I would also like
 * to consider a state machine structure like the RobotHardware code included.
 *
 * The idea is that the Roadrunner Autonomous opMode and the teleOpModes will be both able to refer
 * to this HardwareClasses file so that when hardware changes are made in the future it will need to
 * only be changed in one file. I'm not sure yet how the actions will be used in teleOp yet though.
 */

public class HardwareClasses {

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "lift");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    lift.setPower(0.8);
                }

                double position = lift.getCurrentPosition();
                packet.put("Lift Position", position);
                if (position < 3000) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    lift.setPower(-0.8);
                }

                double position = lift.getCurrentPosition();
                packet.put("Lift Position", position);
                if (position > 100) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    public class IntakeWheels {
        private CRServo intakeLeft;
        private CRServo intakeRight;

        public IntakeWheels(HardwareMap hardwareMap) {
            intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
            intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        }

        public class IntakeSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeLeft.setPower(1);
                intakeRight.setPower(-1);
                return false;
            }
        }

        public Action intakeSample() {
            return new IntakeSample();
        }

        public class OuttakeSample implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(1);
                return false;
            }
        }

        public Action outtakeSample() {
            return new OuttakeSample();
        }

        public class StopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                return false;
            }
        }

        public Action stopIntake() {
            return new StopIntake();
        }
    }

    public class Arm {
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    arm.setPower(0.8);
                }

                double position = arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position < 3000) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armUp(){
            return new ArmUp();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    arm.setPower(-0.8);
                }

                double position = arm.getCurrentPosition();
                packet.put("Arm Position", position);
                if (position > 100) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown(){
            return new ArmDown();
        }
    }
}
