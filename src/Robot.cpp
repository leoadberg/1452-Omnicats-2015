#include "WPILib.h"

// FINAL ROBOT 1 CODE
// Sam Bernstein & Leo Adberg
class Robot: public IterativeRobot
{
private:

	LiveWindow *lw;

	Compressor *c = new Compressor(0);
	Solenoid *suctionCups = new Solenoid(0);
	DoubleSolenoid *piston1 = new DoubleSolenoid(1,2);
	DoubleSolenoid *piston2 = new DoubleSolenoid(3,4);

	Joystick *driveStick = new Joystick(0); // only joyauxStick
	Joystick *auxStick = new Joystick(1);

	DigitalInput *limit = new DigitalInput(2);
	Encoder *liftEncoder_L = new Encoder(0, 1, true);
	Encoder *liftEncoder_R = new Encoder(3, 4, true); // newMotorTest encoder

	CANTalon *leftFront = new CANTalon(1);
	CANTalon *leftBack = new CANTalon(5);
	CANTalon *rightFront = new CANTalon(2);
	CANTalon *rightBack = new CANTalon(6);

	TalonSRX *PWMlf = new TalonSRX(0);
	TalonSRX *PWMlb = new TalonSRX(1);
	TalonSRX *PWMrf = new TalonSRX(2);
	TalonSRX *PWMrb = new TalonSRX(3);

	CANTalon *lift_R = new CANTalon(4);
	CANTalon *lift_L = new CANTalon(3);

	RobotDrive *drive = new RobotDrive(PWMlf,PWMlb,PWMrf,PWMrb);
	Gyro *gyro = new Gyro(0);

	Timer *autonTimer;

	const float cycleWaitTime = .0; // the time it waits between each cycle, in seconds (WE'RE NOT USING THIS)
	// for stepping through for each subsystem in autonomous
	int stepDrive = 0;
	int stepLift = 0;
	int stepPneumatics = 0;

	// Smooth Start elevator variables
	const int toteHeight = 517; // number of encoder ticks per 1 encoder height
	const int stopBuffer = 100; // encoder ticks away from stopping point that smoothStop starts smoothness
	const int maxHeight = 2650; // encoder ticks from very bottom to very top. This is exact.
	const int stopMargin = 200; // encoder ticks distance away from top for stopping (it's a safety buffer)
	const int stopHeight = maxHeight - stopMargin; // sets the stop height

	const float maxLiftSpeed = 0.85;
	const float startSpeed = .2*maxLiftSpeed; // acceleration starts here, skips the slowest part of Smooth Start
	const float startTime = .3; // How long it takes Smooth Start to reach maxLiftSpeed, in seconds
	const float increaseSpeed = (maxLiftSpeed - startSpeed)/(startTime / 0.02);
	float smoothStart = startSpeed;  // motor value during smooth start

	const float hardCorrection = 1.17;
	const float correction = 0.15;
	float correctionDifference = correction*smoothStart;
	int leftEncoder;
	int rightEncoder;
	int gyroValue;
	const int fakeZero = 50;

	// Smooth Start 90-Align variables
	const int northDegrees = 0; // number of encoder ticks per 1 encoder height
	const int eastDegrees = 90; // encoder ticks away from stopping point that smoothStop starts smoothness
	const int southDegrees = 180; // encoder ticks from very bottom to very top. This is exact.
	const int westDegrees = 270; // encoder ticks distance away from top for stopping (it's a safety buffer)

	const float maxAlignSpeed = .3; // don't name ANYTHING this anywhere else or bad stuff will probably happen
	const float startAlignSpeed = .2*maxAlignSpeed; // acceleration starts here, skips the slowest part of Smooth Start
	const float startAlignTime = .3; // How long it takes Smooth Start to reach maxAlignSpeed, in seconds
	const float increaseAlignSpeed = (maxAlignSpeed - startAlignSpeed)/(startAlignTime / 0.02);
	float smoothAlign = startAlignSpeed;  // motor value during smooth align


	//driveStick buttons
	const int westButton = 1;
	const int southButton = 2;
	const int eastButton = 3;
	const int northButton = 4;
	const int resetGyroButton = 10;
	// joysticks are for mecanum Orient Drive



	//auxStick buttons
	const int upButton = 6; // on auxStick
	const int downButton = 8; // on auxStick
	const int pos1Button = 1; // on auxStick
	const int pos2Button = 2; // on auxStick
	const int pos3Button = 3; // on auxStick
	const int pos4Button = 4; // on auxStick
	const int resetEncodersButton = 9; // on auxStick
	const int suctionCupsButton = 5;
	const int pistonButton = 7;

	const float speedM = .8; // default testing drivetrain max speed

	void OutputLift(float reverse, float difference = 0) {
		lift_L->Set(reverse*(smoothStart + difference)*hardCorrection);
		lift_R->Set(-reverse*(smoothStart - difference));
	}
	void OutputLiftRegular(float reverse, float liftSpeed) {
		lift_L->Set(reverse*liftSpeed);
		lift_R->Set(-reverse*liftSpeed);
	}

	void OutputPointTurn(float direction = 1.0, float speedTurn = .4) { // for point turning and 90-align
		rightFront->Set(-direction*speedTurn);
		rightBack->Set(-direction*speedTurn);
		leftFront->Set(direction*speedTurn);
		leftBack->Set(direction*speedTurn);
	}

	void OutputAllDrive(float velocity = .5) {
		rightFront->Set(velocity);
		rightBack->Set(velocity);
		leftFront->Set(velocity);
		leftBack->Set(velocity);
	}

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		gyro->InitGyro();
	}


	void AutonomousInit()
	{
		//testEncoder->Reset();
		//int step = 0;
		autonTimer = new Timer();
		autonTimer->Start();
		gyroValue = 0;
	}

	void AutonomousPeriodic()
	{
		switch (stepDrive)
		{
		case 0:

			if (autonTimer->Get() > 1.0){
				stepDrive++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
		}
		SmartDashboard::PutNumber("Left Encoder", liftEncoder_L->Get());
		SmartDashboard::PutNumber("Right Encoder", liftEncoder_R->Get());
		SmartDashboard::PutBoolean("Limit", limit->Get());
	}

	void TeleopInit()
	{
		//SmartDashboard::init();
		liftEncoder_L->Reset();
		liftEncoder_R->Reset();
		gyro->Reset();
		gyroValue = 0;
		//drive->SetSafetyEnabled(false);
		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor, true);
		drive->SetExpiration(1.0);
		leftFront->SetExpiration(1.0);
		leftBack->SetExpiration(1.0);
		rightFront->SetExpiration(1.0);
		rightBack->SetExpiration(1.0);
	}

	void TeleopPeriodic()
	{

		// Mecanum drive, no gyro
		/*
		rightFront->Set(speedM*(driveStick->GetY() - driveStick->GetZ() - driveStick->GetX()));
		rightBack->Set(speedM*(driveStick->GetY() - driveStick->GetZ() + driveStick->GetX()));
		leftFront->Set(speedM*(driveStick->GetY() + driveStick->GetZ() + driveStick->GetX()));
		leftBack->Set(speedM*(driveStick->GetY() + driveStick->GetZ() - driveStick->GetX()));
*/
/*
		leftFront->SetControlMode(CANSpeedController::kPercentVbus);
		leftBack->SetControlMode(CANSpeedController::kPercentVbus);
		rightFront->SetControlMode(CANSpeedController::kPercentVbus);
		rightBack->SetControlMode(CANSpeedController::kPercentVbus);
		drive->MecanumDrive_Cartesian(speedM*(driveStick->GetX()), speedM*(driveStick->GetY()), speedM*(driveStick->GetZ()));
*/
//		drive->MecanumDrive_Cartesian(driveStick->GetX(),driveStick->GetY(),driveStick->GetZ(), gyro->GetAngle());
//		leftFront->Set(PWMlf->Get());
//		leftBack->Set(PWMlb->Get());
//		rightFront->Set(PWMrf->Get());
//		rightBack->Set(PWMrb->Get());

		//Wait(cycleWaitTime);

		gyroValue = ( ((int)gyro->GetAngle() + 3600000) % 360);

		float dir = 1.0; // for reversing turning direction
		float alignBufferZone = 30.0;
		float minAlignMultiplier = .1;

		if (driveStick->GetRawButton(northButton)) // north = 0, dir = 1 means clockwise is positive
		{
			if (gyroValue > 180) {
				OutputPointTurn((float)(dir * std::min((float)(abs(northDegrees + 360 - gyroValue)/alignBufferZone + minAlignMultiplier), 1.0)), maxAlignSpeed);
			}
			else {
				OutputPointTurn((float)(-dir * std::min((float)abs(northDegrees - gyroValue)/alignBufferZone + minAlignMultiplier, 1.0)), maxAlignSpeed);
			}
		}
		else if (driveStick->GetRawButton(eastButton))
		{
			if (gyroValue > 270 || gyroValue < 90) {
				OutputPointTurn(dir);
			}
			else {
				OutputPointTurn(-dir);
			}
		}
		else if (driveStick->GetRawButton(southButton))
		{
			if (gyroValue < 180) {
				OutputPointTurn(dir);
			}
			else {
				OutputPointTurn(-dir);
			}
		}
		else if (driveStick->GetRawButton(westButton))
		{
			if (gyroValue < 270 && gyroValue > 90) {
				OutputPointTurn(dir);
			}
			else {
				OutputPointTurn(-dir);
			}
		}
		else {
			drive->MecanumDrive_Cartesian(speedM*driveStick->GetX(),speedM*driveStick->GetY(),speedM*driveStick->GetZ(), gyro->GetAngle());
			leftFront->Set(PWMlf->Get());
			leftBack->Set(PWMlb->Get());
			rightFront->Set(PWMrf->Get());
			rightBack->Set(PWMrb->Get());
			//OutputAllDrive(0.0);
		}

		// Reset Encoders in elevator
		if (auxStick->GetRawButton(resetEncodersButton))
		{
			liftEncoder_L->Reset();
			liftEncoder_R->Reset();
		}

		// pneumatics
		suctionCups->Set(auxStick->GetRawButton(suctionCupsButton)); // suction cups

		if (auxStick->GetRawButton(1)){ // pneumatic extender arm acquisition thing
			piston1->Set(DoubleSolenoid::kForward);
			piston2->Set(DoubleSolenoid::kForward);
		} else {
			piston1->Set(DoubleSolenoid::kReverse);
			piston2->Set(DoubleSolenoid::kReverse);
		}


		// elevator lift code
		correctionDifference = correction*smoothStart*std::min((float)abs(leftEncoder-rightEncoder)/50.0 + 0.5,1.0);
		leftEncoder = -1*(liftEncoder_L->Get())+fakeZero;
		rightEncoder = (liftEncoder_R->Get())+fakeZero;


		if ( (auxStick->GetRawButton(downButton)
				|| (auxStick->GetRawButton(pos1Button) && leftEncoder > 0)
				|| (auxStick->GetRawButton(pos2Button) && leftEncoder > toteHeight)
				|| (auxStick->GetRawButton(pos3Button) && leftEncoder > toteHeight*2)
				|| (auxStick->GetRawButton(pos4Button) && leftEncoder > toteHeight*3))) // move down
						{
			if (smoothStart < maxLiftSpeed) {
				smoothStart += increaseSpeed;
			}
			else {
				smoothStart = maxLiftSpeed;
			}

			if (leftEncoder > rightEncoder) {
				OutputLift(-1.0
						* std::min(abs((abs(leftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos3Button))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos4Button))/stopBuffer),1)
								, correctionDifference);
			}
			else if (leftEncoder < rightEncoder) {
				OutputLift(-1.0
						* std::min(abs((abs(leftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos3Button))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos4Button))/stopBuffer),1)
								, -correctionDifference);
			}
			else {
				OutputLift(-1.0);
			}
		}
		else if (auxStick->GetRawButton(upButton)
				|| (auxStick->GetRawButton(pos1Button) && leftEncoder < 0)
				|| (auxStick->GetRawButton(pos2Button) && leftEncoder < toteHeight)
				|| (auxStick->GetRawButton(pos3Button) && leftEncoder < toteHeight*2)
				|| (auxStick->GetRawButton(pos4Button) && leftEncoder < toteHeight*3)) // move up
		{
			if (smoothStart < maxLiftSpeed) {
				smoothStart += increaseSpeed;
			}
			else {
				smoothStart = maxLiftSpeed;
			}

			if (leftEncoder < rightEncoder) {
				OutputLift(1.0
						* std::min(abs( (abs(leftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
								, correctionDifference);
			}
			else if (leftEncoder > rightEncoder) {
				OutputLift(1.0
						* std::min(abs( (abs(leftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)) )/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
				* std::min(abs((abs(leftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
								, -correctionDifference);
			}
			else {
				OutputLift(1.0);
			}
		}
		else if (driveStick->GetRawButton(5)) { //
			OutputLiftRegular(1.0, speedM);
		}
		else if (driveStick->GetRawButton(7)) {
			OutputLiftRegular(-1.0, speedM);
		}
		else {
			//				OutputLift(0.0);

			lift_L->Set(-speedM*auxStick->GetY()); // defaults to regular joyauxStick control of each lift side
			lift_R->Set(speedM*auxStick->GetThrottle());

			smoothStart = startSpeed;
		}


		SmartDashboard::PutNumber("Left Encoder", leftEncoder);
		SmartDashboard::PutNumber("Right Encoder", rightEncoder);
		SmartDashboard::PutBoolean("Limit", limit->Get());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
