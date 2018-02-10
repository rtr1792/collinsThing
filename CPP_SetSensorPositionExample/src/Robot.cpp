#include <iostream>
#include <memory>
#include <string>
#include "CANTalon.h"
#include "WPILib.h"
#include <unistd.h>
#include "ctre/Phoenix.h"
#include <Encoder.h>
class Robot: public frc::IterativeRobot {
	//WPI_TalonSRX *srx = new WPI_TalonSRX(0);
public:

	//WPI_TalonSRX *lr = new WPI_TalonSRX(1);
	CANTalon srx;
	Joystick _joy;
	std::stringstream _work;
	bool _btn1, _btn2, _btn3, _btn4, _btn5;
	/** simple constructor */
	Robot() : srx(1), _joy(0), _work(), _btn1(false), _btn2(false), _btn3(false), _btn4(false), _btn5(false) 	{ }
	/* everytime we enter disable, reinit*/
	void RobotInit() override {
		srx.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative); /* MagEncoder meets the requirements for Unit-Scaling */
		srx.SetStatusFrameRateMs(CANTalon::StatusFrameRateFeedback, 5); /* Talon will send new frame every 5ms */
	};
	/* every loop */
	void TeleopInit() override
		{
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
				int absolutePosition = srx.GetPulseWidthPosition() & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
				/* use the low level API to set the quad encoder signal */
				srx.SetEncPosition(absolutePosition);


				/* choose the sensor and sensor direction */
				srx.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
				srx.SetSensorDirection(false);
				//_srx.ConfigEncoderCodesPerRev(XXX), // if using FeedbackDevice.QuadEncoder
				//_srx.ConfigPotentiometerTurns(XXX), // if using FeedbackDevice.AnalogEncoder or AnalogPot

				/* set the peak and nominal outputs, 12V means full */
				srx.ConfigNominalOutputVoltage(+0., -0.);
				srx.ConfigPeakOutputVoltage(+12., -12.);
				/* set the allowable closed-loop error,
				 * Closed-Loop output will be neutral within this range.
				 * See Table in Section 17.2.1 for native units per rotation.
				 */
				srx.SetAllowableClosedLoopErr(0); /* always servo */
				/* set closed loop gains in slot0 */
				srx.SelectProfileSlot(0);
				srx.SetF(0.0);
				srx.SetP(0.05); //Anything Smaller than 0.01 seems to cause nothing to happen
				srx.SetI(0.0);
				srx.SetD(0.0);
		}
		;

		void TeleopPeriodic() override{
		bool btn1 = _joy.GetRawButton(1);	/* get buttons */
		bool btn2 = _joy.GetRawButton(2);
		bool btn3 = _joy.GetRawButton(3);
		bool btn4 = _joy.GetRawButton(4);
		bool btn5 = _joy.GetRawButton(5);
		double pot = -1*(_joy.GetThrottle()-1);
		double potspeed = pot*4000;
		frc::SmartDashboard::PutNumber("Pot", pot);
		frc::SmartDashboard::PutNumber("Potspeed", potspeed);
		/* on button unpress => press, change pos register */
		if(!_btn1 && btn1) {			srx.Set(potspeed);}
		if(!_btn3 && btn3) {			srx.Set(100.0);}
		if(!_btn5 && btn5) {			srx.Set(0.0);}
		/* remove this and at most we get one stale print (one loop) */
		usleep(10e3);

		/* call get and serialize what we get*/
		double selectpos = srx.GetPosition(); //In Rotations
		double selectvel = srx.GetSpeed(); //Seems to be half as much as set number

		int quadpos = srx.GetEncPosition(); //In Steps out of 4096
		int quadvel = srx.GetEncVel();

		int analograw = srx.GetAnalogInRaw();
		int analogpos = srx.GetAnalogIn();
		int analogvel = srx.GetAnalogInVel();

		frc::SmartDashboard::PutNumber("Selected Encoder Position", selectpos);
		frc::SmartDashboard::PutNumber("Selected Encoder Velocity", selectvel);

		frc::SmartDashboard::PutNumber("Quadrative Position", quadpos);
		frc::SmartDashboard::PutNumber("Quadrative Velocity", quadvel);

		frc::SmartDashboard::PutNumber("Analog Raw", analograw);
		frc::SmartDashboard::PutNumber("Analog Position", analogpos);
		frc::SmartDashboard::PutNumber("Analog Velocity", analogvel);



		if (!_btn4 && btn4) {
			/* Position mode - button just pressed */
			srx.SetControlMode(CANSpeedController::kSpeed);
			srx.Set(1000); /* 50 rotations in either direction */
		}
		if(!_btn2 && btn2) {
			srx.SetControlMode(CANSpeedController::kSpeed);
			srx.Set(0);
		}

	}
};
START_ROBOT_CLASS(Robot)
