/*
 * Gripper.cpp
 *
 *  Created on: Feb 4, 2015
 *      Author: Chiu
 */

#include "Gripper.h"


Gripper::Gripper() {

	connected=false;

	for(int i=0;i<8;i++)
	{
	read[i]=0;
	write[i]=0;
	}

}

Gripper::~Gripper() {

	if(isActivated())
		this->deactivate();

	if(connected)
		this->disconnect();

}

bitset<1> Gripper::Finger::gGTO=bitset<1>::bitset();

Gripper::Finger::Finger() {


}

Gripper::Finger::~Finger() {



}

void Gripper::Finger::setPosition(int position){

	rPR=bitset<8>::bitset(position);

}
void Gripper::Finger::setSpeed(int position){

	rSP=bitset<8>::bitset(position);

}
void Gripper::Finger::setForce(int position){

	rFR=bitset<8>::bitset(position);

}

void Gripper::Finger::clear(void){

	setPosition(0);
	setSpeed(0);
	setForce(0);
}

bool Gripper::Finger::isMoving() const{

	if (Finger::gGTO.to_ulong()==0x1) //gGTO
	 if(gDT.to_ulong()==0x00)
	  return true;

 return false;
}

int Gripper::Finger::getPositionReq() const{

return gPR.to_ulong();

}

int Gripper::Finger::getPosition() const{


return gPO.to_ulong();

}

int Gripper::Finger::getCurrent() const{


return gCU.to_ulong();

}


void Gripper::getBits(void){


	modbus_read_input_registers(mb,0x000,8,read);

	//! Big Endian Convention
	gripper=bitset<8>::bitset(read[0]>>8);
	object=bitset<8>::bitset(read[0]);
	fault=bitset<8>::bitset(read[1]>>8);
	a.gPR=bitset<8>::bitset(read[1]);
    a.gPO=bitset<8>::bitset(read[2]>>8);
    a.gCU=bitset<8>::bitset(read[2]);
    b.gPR=bitset<8>::bitset(read[3]>>8);
    b.gPO=bitset<8>::bitset(read[3]);
    b.gCU=bitset<8>::bitset(read[4]>>8);
    c.gPR=bitset<8>::bitset(read[4]);
    c.gPO=bitset<8>::bitset(read[5]>>8);
    c.gCU=bitset<8>::bitset(read[5]);
    s.gPR=bitset<8>::bitset(read[6]>>8);
    s.gPO=bitset<8>::bitset(read[6]);
    s.gCU=bitset<8>::bitset(read[7]>>8);

    a.gDT[0]=object[0];
    a.gDT[1]=object[1];
    b.gDT[0]=object[2];
    b.gDT[1]=object[3];
    c.gDT[0]=object[4];
    c.gDT[1]=object[5];
    s.gDT[0]=object[6];
    s.gDT[1]=object[7];

    Finger::gGTO[0]=gripper[3];

}

void Gripper::sendBits(void){

	//!Big Endian Convention
	write[0]=action.to_ulong()<<8 |option.to_ulong();
	write[1]=0x00<<8 |a.rPR.to_ulong();
	write[2]=a.rSP.to_ulong()<<8 |a.rFR.to_ulong();
	write[3]=b.rPR.to_ulong()<<8 |b.rSP.to_ulong();
	write[4]=b.rFR.to_ulong()<<8 |c.rPR.to_ulong();
	write[5]=c.rSP.to_ulong()<<8 |c.rFR.to_ulong();
	write[6]=s.rPR.to_ulong()<<8 |s.rSP.to_ulong();
	write[7]=s.rFR.to_ulong()<<8 |0x00;

	modbus_write_registers(mb,0x000,8,write);

}

void Gripper::connect(const char* ip, int port){


	this->mb = modbus_new_tcp(ip, port);

	if (modbus_connect(mb) == -1) {

		cout<< "Connection failed: " << modbus_strerror(errno) <<endl;
	    modbus_free(mb);
	    this->connected=false;


		}else this->connected=true;

}

void Gripper::disconnect(void){

    modbus_close(mb);
	modbus_free(mb);

	this->connected=false;

}

bool Gripper::isConnected(void) const{

    return this->connected;
}

void Gripper::activate(Mode mode){

	this->go(false);


	this->action[0]=1;

		switch(mode){
		case Basic:
			this->action[1]=0;
			this->action[2]=0;
			break;
		case Pinch:
			this->action[1]=1;
			this->action[2]=0;
			break;
		case Wide:
			this->action[1]=0;
			this->action[2]=1;
			break;
		case Scissor:
			this->action[1]=1;
			this->action[2]=1;
			break;
		}

}

void Gripper::setIndividualCtrl(bool rICF, bool rICS){


	b.setPosition(a.gPO.to_ulong());
	b.setForce(a.rFR.to_ulong());
	b.setSpeed(a.rSP.to_ulong());

	c.setPosition(a.gPO.to_ulong());
	c.setForce(a.rFR.to_ulong());
	c.setSpeed(a.rSP.to_ulong());

	s.setPosition(a.gPO.to_ulong());
	s.setForce(a.rFR.to_ulong());
	s.setSpeed(a.rSP.to_ulong());


	this->option[2]=rICF;
	this->option[3]=rICS;




}


void Gripper::setPosition(int position){

a.setPosition(position);

}

void Gripper::setSpeed(int speed){

a.setSpeed(speed);

}

void Gripper::setForce(int force){

a.setForce(force);

}

void Gripper::clear(void){

a.clear();
b.clear();
c.clear();
s.clear();

}



void Gripper::go(bool flag){

	action[3]=flag;

}

void Gripper::deactivate(void){

	go(false);
	clear();

	action[0]=0;

}

void Gripper::emergencyRelease(void){

	this->go(false);
	action[4]=1;
}

void Gripper::synchronise(Sync channel){

	if(connected){

			switch(channel){
					case SendOnly:
					sendBits();
					break;
					case ReadOnly:
					getBits();
					break;
					case Dual:
					sendBits();
					getBits();
					break;
			   	   	   }

				}

}

Gripper::Mode Gripper::getMode(void) const{

	bitset<2> gMOD;
	gMOD[0]=gripper[1];
	gMOD[1]=gripper[2];

	Mode mode;
	                switch(gMOD.to_ulong()){
						case 0:
						mode=Basic;
						break;
						case 1:
						mode=Pinch;
						break;
						case 2:
						mode=Wide;
						break;
						case 3:
						mode=Scissor;
						break;
				   	   	   }
return mode;
}

int Gripper::getPositionReq() const{

return a.getPositionReq();

}

int Gripper::getPosition() const{

return a.getPosition();

}

int Gripper::getCurrent() const{

return a.getCurrent();

}

bool Gripper::isActivated(void) const{

	bitset<2> gIMC;
	gIMC[0]=gripper[4];
	gIMC[1]=gripper[5];

	if(gIMC.to_ulong()==0x03)
		return true;
	else return false;

}

bool Gripper::isMoving(void) const{
	bitset<2> gSTA;
	gSTA[0]=gripper[6];
	gSTA[1]=gripper[7];

	if (gripper[3]==0x1) //gGTO
	 if(gSTA.to_ulong()==0x00)
	 return true;

 return false;
}


Gripper::Fault Gripper::getFaultStatus(void) const{

	bitset<4> gFLT;
	for (int i=0; i<4; i++)
	gFLT[i]=fault[i];

	if(gFLT.to_ulong()==0)
	return NoFault;

	if(gFLT.to_ulong()<8)
	return ActionDelay;

	if(gFLT.to_ulong()<0x0D)
	return Minor;

	return Major;
}

string Gripper::getFaultMsg(void) const{

	bitset<4> gFLT;
	for (int i=0; i<4; i++)
	gFLT[i]=fault[i];

	string msg;

	switch(gFLT.to_ulong()){

	case 0:
	msg="No Fault!";
	break;

	case 5:
	msg="Action delayed: Activation not completed!";
	break;
	case 6:
	msg="Action delayed: Mode change not completed!";
	break;
	case 7:
	msg="Action delayed: Gripper not activated!";
	break;

	case 9:
	msg="Communication chip not ready!";
	break;
	case 0x0A:
	msg="Cannot change mode: interference (less than 20 sec) detected on Scissor!";
	break;
	case 0x0B:
	msg="Emergency release in progress!";
	break;

	case 0x0D:
	msg="Cannot activate gripper: check for interference!";
	break;
	case 0x0E:
	msg="Cannot change mode: interference (more than 20 sec) detected on Scissor!";
	break;
	case 0x0F:
	msg="Emergency release completed: Activation required!";
	break;

	}

return msg;
}
