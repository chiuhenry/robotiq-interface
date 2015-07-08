/*! \mainpage
 *  An Interface (C++ Linux Library - libgripper.a) to communicate with the Robotiq adaptive 3 Fingers gripper.\n
 *  This interface is a communication layer that are built on top of the Modbus TCP Protocol.\n
 *  It is advised that the Robotiq Instruction Manual be consulted before using this interface.\n
 *  A showcase (demo.cpp) and a template (template.cpp) is available.\n
 *  \n
 *  Created on: Jan 30, 2015\n
 *  Author: Chiu
 */

#ifndef GRIPPER_H_
#define GRIPPER_H_

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <bitset>
#include "modbus/modbus.h"

#include <iostream>
using namespace std;


//! Interface to communicate with gripper
class Gripper {

	modbus_t *mb; //!<Modbus TCP Communication Module
	uint16_t read[8], write[8]; //!<register buffer
	bool connected;

	bitset<8> 	action, option;  //!<requests
	bitset<8> 	gripper, object, fault;  //!<status


		void getBits(void);
		void sendBits(void);

public:
	//!gripper operation Mode
	enum Mode {Basic, Pinch, Wide, Scissor};
	//!Synchronisation Channel
	enum Sync {SendOnly, ReadOnly, Dual};
	//!Fault Status
	/*!
	 * NoFault/ActionDelay (led off), Minor (led turns red), Major (led blinking red)
	*/
	enum Fault {NoFault, ActionDelay, Minor, Major};

/*
 * Finger and Scissor Object
 */

		//!Individual Data of fingers and scissor
		class Finger{

					bitset<8> rPR; 	  //!<Position request
					bitset<8> rSP; 	  //!<Speed
					bitset<8> rFR;    //!<Force
					bitset<8> gPR; 	  //!<Position request echo
					bitset<8> gPO;    //!<Actual position
					bitset<8> gCU;    //!<Current
					bitset<2> gDT;    //!<Object detection

					static bitset<1> gGTO; //!<Action Status (of gripper)
					friend class Gripper;

					Finger();
					virtual ~Finger();

					public:
					//!Sets position (range: fully open=0 to fully closed=255) for finger
					void setPosition(int position);
					//!Sets speed (range: min=0 to max=255) for finger
					void setSpeed(int speed);
					//!Sets force (range: min=0 to max=255) for finger
					void setForce(int force);
					//!Sets position, speed and force back to 0 for finger
					void clear(void);
					//!Returns true if finger is in motion
					bool isMoving(void) const;
					//!Returns position requested (Echo) of finger
					int getPositionReq(void) const;
					//!Returns actual position of finger
					int getPosition(void) const;
					//!Returns electric current consumption (0.1*getForce() in mA) of finger
					int getCurrent(void) const;

					}a,b,c,s;//!<objects of individual fingers and scissor

	//!Constructs a Gripper Object
	Gripper();
	 //!Guarantees that the gripper is deactivated and disconnected
	virtual ~Gripper();

/*
 * Gripper Requests
 */


    //!Establishes a TCP Connection to the gripper
	void connect(const char* ip, int port);
	//!Disconnects from the gripper
    void disconnect(void);
	//!Activates gripper in Mode mode
	void activate(Mode mode=Basic);
	//!Enables/disables individual control of fingers/scissor
	/*!
	 * \param bool rICF: Individual control of fingers\n
	 * \param bool rICS: Individual control of scissor\n
	 *\n
	 * caution: position request, speed and force for ALL fingers will be initialised, resepctively to\n
	 * the current position, speed and force of gripper.\n
	 * option rICF is discarded by gripper if mode is set to Scissor and option rICS is set to false\n
	 * mode is discarded by gripper if option rICS is set to true
	 */
	void setIndividualCtrl(bool rICF, bool rICS=false);
	//!Deactivates gripper
	void deactivate(void);
	//!Automatic Release routine
	/*!
     * usage: to disengage gripper after an emergency stop of robot\n
     * not to be used under normal operating conditions
	 */
	void emergencyRelease(void);
	//!Synchronises data between Interface and gripper according to the specified channel Sync
	/*!
	 * data are NOT exchanged with the gripper until this method is called\n
	 * this method does nothing until the gripper is connected.\n
	 * usage: to be implemented in a real time communication thread\n
	 * method can also be called in a one shot manner (i.e. threadfree/call at will)\n
	 * maximum calling frequency: 1 millisecond
	 */
    void synchronise(Sync Channel=Dual);
	//!Sets position (range: fully open=0 to fully closed=255) for gripper
    void setPosition(int position);
    //!Sets speed (range: min=0 to max=255) for gripper
    void setSpeed(int speed);
    //!Sets force (range: min=0 to max=255) for gripper
    void setForce(int force);
    //!Sets position, speed and force back to 0 for all fingers and scissor
    /*!
     * clear() is implemented in Gripper::deactivate()
     */
    void clear(void);
    //!Go2 requested position (flag=true) or Stop (flag=false)
    /*!
     * go(false) is implemented in Gripper::activate(Mode), Gripper::deactivate()\n
     * and Gripper::emergencyRelease()
     */
	void go(bool flag);

/*
 * Gripper Status
 */

	//!Returns true if gripper is connected
	bool isConnected(void) const;
	//!Returns true if gripper has been activated in the corresponding mode and ready for command
	bool isActivated(void) const;
	//!Returns true if gripper is in motion towards requested position
	bool isMoving(void) const;
	//!Returns gripper operation Mode mode
	Gripper::Mode getMode(void) const;
	//!Returns position requested (Echo) of gripper
	int getPositionReq(void) const;
	//!Returns actual position of gripper
	int getPosition(void) const;
	//!Returns electric current consumption (0.1*getForce() in mA) of gripper
	int getCurrent(void) const;

/*
 * Fault
 */

	//!Returns Fault Status
	Gripper::Fault getFaultStatus(void) const;
	//!Returns fault message
	string getFaultMsg(void) const;




};

#endif /* GRIPPER_H_ */
