#include "WPILib.h"

// FINAL ROBOT 1 CODE
// Sam Bernstein && Leo Adberg 2015 FRC
// "This game is garbage, and our robot sucks."
class Robot: public IterativeRobot
{
private:

	LiveWindow *lw;

	Compressor *c = new Compressor(0);
	Solenoid *suctionCups = new Solenoid(0);
	DoubleSolenoid *piston1 = new DoubleSolenoid(1,2);
	//DoubleSolenoid *piston2 = new DoubleSolenoid(3,4);

	Joystick *driveStick = new Joystick(0); // only joyauxStick
	Joystick *auxStick = new Joystick(1);

	DigitalInput *topLimit_L = new DigitalInput(2); // for stopping elevator at top
	DigitalInput *bottomLimit_L = new DigitalInput(7); // for stopping elevator at bottom

	Ultrasonic *ultrasonic_L = new Ultrasonic(6, 7); // CHANGE PORTS
	Ultrasonic *ultrasonic_R = new Ultrasonic(13, 14); // CHANGE PORTS

	Encoder *liftEncoder_L = new Encoder(0, 1, true);
	Encoder *liftEncoder_R = new Encoder(3, 4, true); // newMotorTest encoder

	Encoder *leftFrontEncoder = new Encoder(5, 6, true);
	Encoder *rightFrontEncoder = new Encoder(7, 8, true);
	Encoder *leftBackEncoder = new Encoder(9, 10, true);
	Encoder *rightBackEncoder = new Encoder(11, 12, true);

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
	int l_LiftEncoder;
	int r_LiftEncoder;

	int r_frontEncoder;
	int l_frontEncoder;
	int r_backEncoder;
	int l_backEncoder;

	float driveInchesPerTick = .4;

	int intermediateGyro;
	float gyroValue;
	const int fakeZero = 50;
	//

	// Smooth Start AND Smooth Stop 90-Align variables
	const float northDegrees = 0.0; // number of encoder ticks per 1 encoder height
	const float eastDegrees = 90.0; // encoder ticks away from stopping point that smoothStop starts smoothness
	const float southDegrees = 180.0; // encoder ticks from very bottom to very top. This is exact.
	const float westDegrees = 270.0; // encoder ticks distance away from top for stopping (it's a safety buffer)
	const float maxDegrees = 4.0*eastDegrees;

	const float maxAlignSpeed = .4; // don't name ANYTHING this anywhere else or bad stuff will probably happen
	const float startAlignSpeed = .2*maxAlignSpeed; // acceleration starts here, skips the slowest part of Smooth Start
	const float startAlignTime = .5; // How long it takes Smooth Start to reach maxAlignSpeed, in seconds
	const float increaseAlignSpeed = (maxAlignSpeed - startAlignSpeed)/(startAlignTime / 0.02);
	float smoothAlign = startAlignSpeed;  // motor value multiplication during smooth align

	float dir = 1.0; // for reversing turning direction
	float alignBufferZone = 30.0;
	float minAlignMultiplier = 0.1;

	//driveStick buttons
	const int westButton = 1;
	const int southButton = 2;
	const int eastButton = 3;
	const int northButton = 4;
	const int acqStart_L = 10; // change
	const int acqStart_R = 11; // change
	const int acqStop = 9;
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


	int autoNumber = 0;
	// for stepping through the steps for each subsystem in autonomous
	int stepDrive = 0;
	int stepLift = 0;
	int stepPneumatics = 0;

	// function global variables:
	//OutputStraightDrive: go forward and backward straight with encoders
	float speedStraightDrive = .3;

	float leftFrontEncoderAuto = 0;
	float rightFrontEncoderAuto = 0;
	float avgDriveEnc = 0;

	bool firstCall = true;

	// AcqGetTote:  grab, suck and bring in tote
	Timer *getToteTimer;
	int stepGetTote = 0;
	float getToteLastTime = 0.0;
	const float getToteSuckTime = 0.3;
	const float getToteExtendTime = 2.0;

	// AcqRoutine: acquisition routine
	int stepAcq = 0;
	int acqRunning = 0; // 0 is not running, -1 is starting from right, 1 is starting from left
	float ultrasonic_L_dist = 0.0;
	float ultrasonic_R_dist = 0.0;

	float r_FrontEncoder_1 = 0.0;
	float l_FrontEncoder_1 = 0.0;
	float r_BackEncoder_1 = 0.0;
	float l_BackEncoder_1 = 0.0;

	float r_FrontDelta = 0.0;
	float l_FrontDelta = 0.0;
	float r_BackDelta = 0.0;
	float l_BackDelta = 0.0;

	float avgStrafeTicks = 0.0;

	const float closerTote_dist = 13.0; // inches, for recognizing that ultrasonic has reached edge of tote (depth)
	const float acqStrafeSpeed = 0.4;
	const float toteDepth_dist = 24.0; // inches, for optimal distance away from tote for acquisition (depth)
	const float toteDepth_range = 2.5; // inches, tolerance for getting within range of tote

	float levelAlign_dist = 4.0; // inches to the side robot must strafe to line up with tote levels


	void SetDriveEncAuto() {
		if (firstCall) {
			leftFrontEncoderAuto = leftFrontEncoder->Get();
			rightFrontEncoderAuto = rightFrontEncoder->Get();

			firstCall = false;
		}

		avgDriveEnc = 0.5*(leftFrontEncoderAuto  + rightFrontEncoderAuto);
	}


	void OutputLift(float reverse, float difference = 0) {
		lift_L->Set(reverse*(smoothStart + difference)*hardCorrection);
		lift_R->Set(-reverse*(smoothStart - difference));
	}
	void OutputLiftRegular(float reverse, float liftSpeed) {
		lift_L->Set(reverse*liftSpeed);
		lift_R->Set(-reverse*liftSpeed);
	}

	void OutputAllDrive(float velocity = .5) {
		rightFront->Set(velocity);
		rightBack->Set(velocity);
		leftFront->Set(velocity);
		leftBack->Set(velocity);
	}

	void OutputPointTurn(float direction, float speedTurn = .4) { // for point turning and 90-align
		rightFront->Set(-direction*speedTurn);
		rightBack->Set(-direction*speedTurn);
		leftFront->Set(direction*speedTurn);
		leftBack->Set(direction*speedTurn);
	}

	void OutputStraightDrive(float direction, float distance = 1.0) { // for encoder AUTO go straight forward/backward movements
		SetDriveEncAuto();

		if (avgDriveEnc*driveInchesPerTick < distance) {
			rightFront->Set(direction*speedStraightDrive);
			rightBack->Set(direction*speedStraightDrive);
			leftFront->Set(direction*speedStraightDrive);
			leftBack->Set(direction*speedStraightDrive);
		}
		else {
			firstCall = true;
			stepDrive++;
		}
	}

	void OutputStrafe(float direction, float speedStrafe = .4) { // for point turning and 90-align
		rightFront->Set(direction*speedStrafe);
		leftBack->Set(direction*speedStrafe);

		rightBack->Set(-direction*speedStrafe);
		leftFront->Set(-direction*speedStrafe);
	}

	bool AcqGetTote() {
		switch(stepGetTote)
		{
		case 0:
			getToteTimer = new Timer();
			getToteTimer->Reset();
			getToteTimer->Start();
			suctionCups->Set(true);
			if (getToteTimer->Get() > getToteSuckTime) {
				stepGetTote++;
				getToteLastTime += getToteTimer->Get();
			}
			break;

		case 1:
			piston1->Set(DoubleSolenoid::kForward);
			if (getToteTimer->Get() > getToteLastTime + getToteExtendTime) {
				stepGetTote++;
				getToteLastTime += getToteTimer->Get();
			}
			break;

		case 2: // retract with tote
			piston1->Set(DoubleSolenoid::kReverse);
			if (getToteTimer->Get() > getToteLastTime + 2.0*getToteExtendTime) { // takes longer to retract
				stepGetTote = 0;
				getToteLastTime = 0; // reset to 0 for next run
				return true;
			}
			break;
		}
		return false;
	}

	void AcqInitialize() {
		ultrasonic_L_dist = ultrasonic_L->GetRangeInches();
		ultrasonic_R_dist = ultrasonic_R->GetRangeInches();
		stepAcq = 0; // reset to first step
	}

	void AcqRoutine() {

		switch (stepAcq)
		{
		case 0: // strafe right until ultrasonic sees tote
			OutputStrafe((float)acqRunning, acqStrafeSpeed); // strafe right

			if (acqRunning == 1) // from left side
			{
				if (ultrasonic_L->GetRangeInches() < ultrasonic_L_dist - closerTote_dist) {
					r_FrontEncoder_1 = rightFrontEncoder->Get(); // store each encoder for reference for next step
					l_FrontEncoder_1 = leftFrontEncoder->Get();
					r_BackEncoder_1 = rightBackEncoder->Get();
					l_BackEncoder_1 = leftBackEncoder->Get();
					stepAcq++;
				}
			}
			else // from right side
			{
				if (ultrasonic_R->GetRangeInches() < ultrasonic_R_dist - closerTote_dist) {
					r_FrontEncoder_1 = rightFrontEncoder->Get(); // store each encoder for reference for next step
					l_FrontEncoder_1 = leftFrontEncoder->Get();
					r_BackEncoder_1 = rightBackEncoder->Get();
					l_BackEncoder_1 = leftBackEncoder->Get();
					stepAcq++;
				}
			}
			break;
		case 1: // strafe right to align with levels
			OutputStrafe(float(acqRunning), acqStrafeSpeed);

			r_FrontDelta = abs(rightFrontEncoder->Get() - r_FrontEncoder_1);
			l_FrontDelta = abs(leftFrontEncoder->Get() - l_FrontEncoder_1);
			r_BackDelta = abs(rightBackEncoder->Get() - r_BackEncoder_1);
			l_BackDelta = abs(leftBackEncoder->Get() - l_BackEncoder_1);

			avgStrafeTicks = 0.25*(r_FrontDelta + l_FrontDelta + r_BackDelta + l_BackDelta);

			if (avgStrafeTicks*driveInchesPerTick > levelAlign_dist) {
				stepAcq++;
			}
			break;
		case 2: // move backward/forward until right distance
			OutputStraightDrive(1.0, acqStrafeSpeed);
			if (abs(ultrasonic_L->GetRangeInches() - toteDepth_dist) < toteDepth_range) {
				stepAcq++;
			}
			break;
		case 3:
			OutputAllDrive(0.0); // stop drive train
			Wait(.4);
			stepAcq++;
			break;
		case 4:
			if (AcqGetTote()) { // run AcqGetTote, if done, end everything and reset
				acqRunning = 0;
			}
			break;

		default:
			while (autonTimer->Get() < 15.0){

			}
		}
	}

	float AlignComparison(float angleIs, float angleTo) { // if first is less than second, return 1.0
		if (angleIs < angleTo) {
			return 1.0;
		}
		return 0.0;
	}

	void AutoForward() { // the most basic default AUTO routine
		switch (stepDrive) // for drive train
		{
		case 0:
			OutputStraightDrive(1.0, 4.0);
			if (autonTimer->Get() > 15.0){
				stepDrive++;
			}
			break;
		case 1:
			OutputAllDrive(0.0);
			if (autonTimer->Get() > 15.0) {
				stepDrive++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
		}

		switch (stepLift) // for elevator
		{
		case 0:
			if (autonTimer->Get() > 15.0){
				stepLift++;
			}
			break;

		default:
			while (autonTimer->Get() < 15.0) {

			}
		}
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

		if (SmartDashboard::GetBoolean("New Name", false)) {
			autoNumber = 1;
		}
		else if (SmartDashboard::GetBoolean("DB/Button2", false)) {
			autoNumber = 2;
		}
		else if (SmartDashboard::GetBoolean("DB/Button3", false)) {
			autoNumber = 3;
		}
		else if (SmartDashboard::GetBoolean("DB/Button4", false)) {
			autoNumber = 4;
		}

		autonTimer = new Timer();
		autonTimer->Start();
		gyroValue = 0;
	}

	void AutonomousPeriodic()
	{
		intermediateGyro = ((int)gyro->GetAngle() + 3600000) % 360;
		gyroValue = (float)intermediateGyro; // it's a FLOAT

		// for drivetrain

		switch(autoNumber)
		{
		case 0:
			AutoForward();
			break;

		case 1:
			AutoForward();
			break;
		}
		SmartDashboard::PutNumber("Left Encoder", liftEncoder_L->Get());
		SmartDashboard::PutNumber("Right Encoder", liftEncoder_R->Get());
		SmartDashboard::PutBoolean("Limit", ultrasonic_L->GetRangeInches());
	}

	void TeleopInit()
	{
		//SmartDashboard::init();
		liftEncoder_L->Reset();
		liftEncoder_R->Reset();
		gyro->Reset();
		gyroValue = 0;

		ultrasonic_L->SetAutomaticMode(true);

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

		intermediateGyro = ((int)gyro->GetAngle() + 3600000) % 360;
		gyroValue = (float)intermediateGyro; // it's a FLOAT

		l_LiftEncoder = -1*(liftEncoder_L->Get()) + fakeZero;
		r_LiftEncoder = (liftEncoder_R->Get()) + fakeZero;

		r_frontEncoder = rightFrontEncoder->Get();
		l_frontEncoder = leftFrontEncoder->Get();
		r_backEncoder = rightBackEncoder->Get();
		l_backEncoder = leftBackEncoder->Get();

		// 90-align Smooth Start and Stop
		if (driveStick->GetRawButton(acqStart_L)) {
			acqRunning = 1;
			stepAcq = 0; // reset to beginning of routine
			AcqInitialize();
		}
		else if (driveStick->GetRawButton(acqStart_R)) {
			acqRunning = -1;
			stepAcq = 0; // reset to beginning of routine
			AcqInitialize();
		}

		if (driveStick->GetRawButton(acqStop)) {
			acqRunning = 0;
			stepAcq = 0;
			stepGetTote = 0;
		}

		if (acqRunning != 0) {
			AcqRoutine();
		}
		else
		{
			if (driveStick->GetRawButton(northButton)) // north = 0, dir = 1 means clockwise is positive
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue > southDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign * (float)std::min((float)(abs((float)northDegrees + (float)360.0 - (float)gyroValue)/((float)alignBufferZone) + (float)minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn( (float)(-dir) * smoothAlign *(float)std::min((float)abs((float)northDegrees - (float)gyroValue)/(float)alignBufferZone + (float)minAlignMultiplier, (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(eastButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue > westDegrees || gyroValue < eastDegrees) {
					//OutputPointTurn(  (float)dir * (float)std::min( AlignComparison(gyroValue, northDegrees) * (float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
					OutputPointTurn(  (float)dir * smoothAlign *(float)std::min((float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else { // if between 90 and 270
					OutputPointTurn(  (float)(-dir) * smoothAlign *(float)std::min( (float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(southButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue < southDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign *(float)std::min( (float)(abs( (southDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn(  (float)(-dir) * smoothAlign *(float)std::min( (float)(abs( (southDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(westButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue < westDegrees && gyroValue > eastDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign * (float)std::min( (float)(abs( (westDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn(  (float)(-dir) * smoothAlign * (float)std::min( (float)(abs( (westDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else {
				drive->MecanumDrive_Cartesian(speedM*driveStick->GetX(),speedM*driveStick->GetY(),speedM*driveStick->GetZ(), gyro->GetAngle());
				leftFront->Set(PWMlf->Get());
				leftBack->Set(PWMlb->Get());
				rightFront->Set(PWMrf->Get());
				rightBack->Set(PWMrb->Get());

				smoothAlign = startAlignSpeed;
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
				//piston2->Set(DoubleSolenoid::kForward);
			} else {
				piston1->Set(DoubleSolenoid::kReverse);
				//piston2->Set(DoubleSolenoid::kReverse);
			}

			// elevator lift code with levels, smooth start and smooth stop

			correctionDifference = correction*smoothStart*std::min((float)abs(l_LiftEncoder-r_LiftEncoder)/50.0 + 0.5,1.0);

			if ( !(topLimit_L->Get()) && (auxStick->GetRawButton(upButton)
					|| (auxStick->GetRawButton(pos1Button) && l_LiftEncoder < 0)
					|| (auxStick->GetRawButton(pos2Button) && l_LiftEncoder < toteHeight)
					|| (auxStick->GetRawButton(pos3Button) && l_LiftEncoder < toteHeight*2)
					|| (auxStick->GetRawButton(pos4Button) && l_LiftEncoder < toteHeight*3))) // move up
			{
				if (smoothStart < maxLiftSpeed) {
					smoothStart += increaseSpeed;
				}
				else {
					smoothStart = maxLiftSpeed;
				}

				if (l_LiftEncoder < r_LiftEncoder) {
					OutputLift(1.0
							* std::min(abs( (abs(l_LiftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
					, correctionDifference);
				}
				else if (l_LiftEncoder > r_LiftEncoder) {
					OutputLift(1.0
							* std::min(abs( (abs(l_LiftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)) )/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
					, -correctionDifference);
				}
				else {
					OutputLift(1.0);
				}
			}
			else if ( !(bottomLimit_L->Get()) && (auxStick->GetRawButton(downButton)
					|| (auxStick->GetRawButton(pos1Button) && l_LiftEncoder > 0)
					|| (auxStick->GetRawButton(pos2Button) && l_LiftEncoder > toteHeight)
					|| (auxStick->GetRawButton(pos3Button) && l_LiftEncoder > toteHeight*2)
					|| (auxStick->GetRawButton(pos4Button) && l_LiftEncoder > toteHeight*3))) // move down
			{
				if (smoothStart < maxLiftSpeed) {
					smoothStart += increaseSpeed;
				}
				else {
					smoothStart = maxLiftSpeed;
				}

				if (l_LiftEncoder > r_LiftEncoder) {
					OutputLift(-1.0
							* std::min(abs((abs(l_LiftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos3Button))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos4Button))/stopBuffer),1)
					, correctionDifference);
				}
				else if (l_LiftEncoder < r_LiftEncoder) {
					OutputLift(-1.0
							* std::min(abs((abs(l_LiftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos3Button))/stopBuffer),1)
					* std::min(abs((abs(l_LiftEncoder)-2*toteHeight*(int)auxStick->GetRawButton(pos4Button))/stopBuffer),1)
					, -correctionDifference);
				}
				else {
					OutputLift(-1.0);
				}
			}

			else if (!(topLimit_L->Get()) && driveStick->GetRawButton(5)) { // move up
				OutputLiftRegular(1.0, speedM);
			}
			else if (!(bottomLimit_L->Get()) && driveStick->GetRawButton(7)) {
				OutputLiftRegular(-1.0, speedM);
			}
			else {
				//				OutputLift(0.0);

				lift_L->Set(-speedM*auxStick->GetY()); // defaults to regular joyauxStick control of each lift side
				lift_R->Set(speedM*auxStick->GetThrottle());

				smoothStart = startSpeed;
			}

		}

		SmartDashboard::PutNumber("Left lift Enc", l_LiftEncoder);
		SmartDashboard::PutNumber("Right lift Enc", r_LiftEncoder);
		SmartDashboard::PutBoolean("Left Ultrasonic", ultrasonic_L->GetRangeInches());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
