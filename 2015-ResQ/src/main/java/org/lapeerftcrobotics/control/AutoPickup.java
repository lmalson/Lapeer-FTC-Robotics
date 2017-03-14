package org.lapeerftcrobotics.control;

/**
 * Created by User on 2/7/2016.
 */
public class AutoPickup {
    private final static double SERVO_ARM_UP = 0.685; // 0.95
    public final static double RIGHT_GRABBER_CLOSED = 0.15; // 0.02
    public final static double SERVO_ARM_DOWN = 0.02;

    private final static double SERVO_GRABBER_INCREMENT = 0.004; // 0.0025
    private final static double SERVO_ARM_INCREMENT_VERY_SLOW = 0.0014; // 0.0012
    private final static double SERVO_ARM_INCREMENT_SLOW = 0.0019; // 0.0017
    private final static double SERVO_ARM_INCREMENT_FAST = 0.0029; // 0.0027

    private double rightGrabberTarget = RIGHT_GRABBER_CLOSED;
    private double rightGrabberServoPos = RIGHT_GRABBER_CLOSED;
    private double armAngleTarget = 0.02;
    private double armAngleServoPos = 0.02;
    private boolean rightGrabberAtTarget = false;
    private boolean armAngleAtTarget = false;
    private double servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;

    private int state = 0;
    private int stateCnt = 0;

    private boolean runForward = false;
    private boolean runBackward = false;
    private boolean unload = false;
    private boolean startRunningBackward = false;

    private boolean testAutonReset = false;
    private boolean testAutonRaise = false;
    private boolean testAutonRelease = false;

    public AutoPickup() {

    }

    public void setRunForward(boolean r) {
        this.runForward = r;
    }

    public void setRunBackward(boolean r) {
        if (!this.runBackward && r) {
            startRunningBackward = true;
        } else {
            startRunningBackward = false;
        }

        this.runBackward = r;
    }

    public void setUnload(boolean u) {
        this.unload = u;
    }

    public int getState() {
        return this.state;
    }

    private void runStateMachine() {


        int nextState = state;
        switch(state) {
            case -20: // transition from unloading
                servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; // 0.99;
                if (rightGrabberAtTarget && stateCnt > 10)
                    nextState = -10;
                break;
            case -10:
                armAngleTarget = 0.5;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED;
                if (armAngleAtTarget && stateCnt > 10)
                    nextState = 0;
                break;
            case 0:
                armAngleTarget = 0.5;
                rightGrabberTarget = 0.5;
                servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
                if (armAngleAtTarget && this.runForward && stateCnt > 10)
                    nextState = 10;
                break;
            case 10: // lower grabbers
                armAngleTarget = 0.02; // 0.02
                rightGrabberTarget = 0.5;
                servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
                if (armAngleAtTarget && runForward && stateCnt > 10)
                    nextState = 20;
                break;
            case 20: // close grabbers
                armAngleTarget = 0.02;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; // 0.99;
                servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
                if (startRunningBackward)
                    nextState = 10;
                else if (armAngleAtTarget && rightGrabberAtTarget && runBackward && stateCnt > 10)
                    nextState = 10;
                else if (rightGrabberAtTarget && runForward && stateCnt > 10)
                    nextState = 30;
                break;
            case 30: // raise grabbers
                armAngleTarget = SERVO_ARM_UP;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; //0.99;
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (startRunningBackward)
                    nextState = 20;
                else if (rightGrabberAtTarget && armAngleAtTarget && runBackward && stateCnt > 10)
                    nextState = 20;
                else if (armAngleAtTarget && runForward && stateCnt > 10)
                    nextState = 40;
                break;
            case 40: // unload  // only open right servo arm
                armAngleTarget = SERVO_ARM_UP;
                rightGrabberTarget = 0.5;
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (rightGrabberAtTarget && runForward && stateCnt > 10)
                    nextState = 50;
                break;
            case 50: // close grabbers
                armAngleTarget = SERVO_ARM_UP;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; //0.99;
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (rightGrabberAtTarget && runForward && stateCnt > 10)
                    nextState = 60;
                break;
            case 60: // lower arm angle with closed grabbers
                armAngleTarget = 0.5;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; //0.99;
                servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
                if (armAngleAtTarget && runForward && stateCnt > 10)
                    nextState = 0;
                break;


            case 100: // close grabbers
                armAngleTarget = 0.5;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; // 0.8
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 110;
                break;
            case 110: // raise grabbers
                armAngleTarget = SERVO_ARM_UP;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED; // 0.8
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 120;
                break;
            case 120: // open grabbers
                rightGrabberTarget = 0.39; // 0.42
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 130;
                break;
            case 130: // lower grabbers
                armAngleTarget = 0.49;  // 0.5
                servoArmSpeed = SERVO_ARM_INCREMENT_VERY_SLOW;
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 140;
                break;
            case 140: // hold grabbers
                servoArmSpeed = SERVO_ARM_INCREMENT_VERY_SLOW;

                if (stateCnt == 20)
                    armAngleTarget = 0.7;
                else if (stateCnt == 50)
                    armAngleTarget = 0.49; // .5
                else if (stateCnt == 100)
                    armAngleTarget = 0.7;
                else if (stateCnt == 150)
                    armAngleTarget = 0.49; // .5
                else if (stateCnt == 200)
                    armAngleTarget = 0.7;
                else if (stateCnt == 250)
                    armAngleTarget = 0.49; // .5
//                else if (stateCnt == 300)
//                    armAngleTarget = 0.7;
//                else if (stateCnt == 350)
//                    armAngleTarget = 0.5;

                if (stateCnt > 300) // 400
                    nextState = 150;
                break;
            case 150: // raise grabbers
                armAngleTarget = SERVO_ARM_UP;
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 160;
                break;
            case 160: // raise grabbers
                armAngleTarget = 0.5;
                rightGrabberTarget = RIGHT_GRABBER_CLOSED;
                servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
                if (armAngleAtTarget && rightGrabberAtTarget && stateCnt > 10)
                    nextState = 0;
                break;

        }
        if (nextState != state) {
            state = nextState;
            stateCnt = 0;
        } else {
            stateCnt++;
        }
    }

    public void process() {
/*
        if (testAutonReset) {
            armAngleTarget = 0.02;
            rightGrabberTarget = RIGHT_GRABBER_CLOSED;
            servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
        }
        else if (testAutonRaise) {
            armAngleTarget = 0.68;
            rightGrabberTarget = RIGHT_GRABBER_CLOSED;
            servoArmSpeed = SERVO_ARM_INCREMENT_SLOW;
        }
        else if (testAutonRelease) {
            armAngleTarget = 0.55;
            rightGrabberTarget = RIGHT_GRABBER_CLOSED;
            servoArmSpeed = SERVO_ARM_INCREMENT_FAST;
            if (armAngleAtTarget) {
                rightGrabberTarget = 0.39;
            }
        }
*/
        if (this.runBackward || this.runForward || this.unload) {

            if (this.unload) {
                if (this.state < 100) {
                    this.state = 100;
                }
            } else {
                if (this.state >= 100) {
                    this.state = -20;
                }
            }

            runStateMachine();

        }

            if (rightGrabberServoPos < rightGrabberTarget) {
                rightGrabberAtTarget = false;
                rightGrabberServoPos += SERVO_GRABBER_INCREMENT;
                if (rightGrabberServoPos >= rightGrabberTarget)
                    rightGrabberServoPos = rightGrabberTarget;
            }
            else if (rightGrabberServoPos > rightGrabberTarget) {
                rightGrabberAtTarget = false;
                rightGrabberServoPos -= SERVO_GRABBER_INCREMENT;
                if (rightGrabberServoPos <= rightGrabberTarget)
                    rightGrabberServoPos = rightGrabberTarget;
            }
            else {
                rightGrabberAtTarget = true;
                rightGrabberServoPos = rightGrabberTarget;
            }

            if (armAngleServoPos < armAngleTarget) {
                armAngleAtTarget = false;
                armAngleServoPos += servoArmSpeed;
                if (armAngleServoPos >= armAngleTarget)
                    armAngleServoPos = armAngleTarget;
            }
            else if (armAngleServoPos > armAngleTarget) {
                armAngleAtTarget = false;
                armAngleServoPos -= servoArmSpeed;
                if (armAngleServoPos <= armAngleTarget)
                    armAngleServoPos = armAngleTarget;
            }
            else {
                armAngleAtTarget = true;
                armAngleServoPos = armAngleTarget;
            }


    }

    public double getRightGrabberServoPos() {
        return this.rightGrabberServoPos;
    }

    public double getArmAngleServoPos() {
        return this.armAngleServoPos;
    }

    public void setTestAutonReset(boolean b) {
        this.testAutonReset = b;
    }

    public void setTestAutonRelease(boolean b) {
        this.testAutonRelease = b;
    }

    public void setTestAutonRaise(boolean b) {
        this.testAutonRaise = b;
    }
}
