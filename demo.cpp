//============================================================================
// Name        : demo.cpp
// Author      : Chiu
// Description : Robotiq Adaptive 3 Finger Demo
//============================================================================

#include "Gripper.h"
#include <pthread.h>
#define sec 1000000 //!<sec in microseconds
#define ms  1000    //!<ms in microseconds

//!thread purposes
enum Thread {Com, Fault};
pthread_t launch(Gripper*, Thread);  //!<thread launcher
void terminate(pthread_t);	 //!<thread terminator

int main() {

	Gripper gripper;
	pthread_t com=launch(&gripper,Com); //!<start communication
	pthread_t fault=launch(&gripper,Fault); //!<start fault monitoring

	//!Connect
	while(!gripper.isConnected())
	{
		gripper.connect("192.168.1.11",502);
		usleep(sec);

	}cout << "connected" <<endl;

	//!Activation
	cout << "Activating in Basic mode" <<endl;
	while(!gripper.isActivated()){

		gripper.activate();

		usleep(sec);

	}cout << "activated" <<endl<<endl;

	//!Mode Change
	cout<<"Perform mode change: 2 Wide, proceed?";
	cin.get();

	gripper.setSpeed(255); //!<set mode chaning speed
	gripper.activate(Gripper::Wide);

			while(gripper.getMode()!=Gripper::Wide){

							usleep(ms);

							}cout<<"changing mode"<<endl;

			while(!gripper.isActivated()){

							usleep(ms);

							}cout<<"mode changed"<<endl<<endl;


	//!Decelerate full close
	cout << "Perform a decelerate full close, proceed?";
	cin.get();
	gripper.setPosition(255);
	gripper.go(true);


			while(!gripper.isMoving()){

							usleep(ms);

							}cout<<"decelerating 2 requested position"<<endl;


			while(gripper.isMoving()){

							if(gripper.getPosition()>=255*0.3)
								if(gripper.getPosition()<255*0.4)
									gripper.setSpeed(255*0.8);

							if(gripper.getPosition()>=255*0.4)
								if(gripper.getPosition()<255*0.5)
									gripper.setSpeed(255*0.6);

						    if(gripper.getPosition()>=255*0.5)
						    	if(gripper.getPosition()<255*0.6)
						    		gripper.setSpeed(255*0.4);

						    if(gripper.getPosition()>=255*0.6)
						    	if(gripper.getPosition()<255*0.7)
						    		gripper.setSpeed(255*0.2);

							if(gripper.getPosition()>=255*0.7)
								if(gripper.getPosition()<255*0.8)
									gripper.setSpeed(0);

							usleep(ms);

							}cout<<"position reached"<<endl<<endl;


	//!Advanced feature
	cout << "Perform individual control of fingers, proceed?" ;
	cin.get();

	gripper.go(false);
	gripper.setIndividualCtrl(true);
	gripper.a.setPosition(70);
	gripper.go(true);


			while(!gripper.isMoving()){

							 usleep(ms);

							 }cout<<"Opening finger A"<<endl;

			while(gripper.isMoving()){

							 usleep(ms);

							 }cout<<"position reached"<<endl<<endl;


	//!Restore and Deactivate
	cout << "Demo ends, deactivate?";
	cin.get();

	gripper.go(false);
	gripper.clear();
	gripper.go(true);

			while(!gripper.isMoving()){

							  usleep(ms);

							 }cout<<"moving back to full open"<<endl;

			while(gripper.isMoving()){

							  usleep(ms);

							 }cout<<"position reached"<<endl;


	gripper.deactivate();
	cout<<"deactivated"<<endl;
	gripper.disconnect();
	cout<<"disconnected"<<endl;

	terminate(fault);
	terminate(com);
	return 0;
}



void* com(void* gripper){

	while(true)
	{
		((Gripper*)gripper)->synchronise(Gripper::Dual);

		usleep(ms);
	}

}

void* fault(void* gripper){

	while(true)
	{
		if(((Gripper*)gripper)->getFaultStatus()!=Gripper::NoFault)
		cout << ((Gripper*)gripper)->getFaultMsg() <<endl;

		usleep(sec);
	}

}



pthread_t launch(Gripper* gripper, Thread type ){

	pthread_t tid;

	if(type==Com)
	pthread_create(&tid, NULL,com,(void*)gripper);
	else pthread_create(&tid, NULL,fault,(void*)gripper);

	return tid;
}

void terminate(pthread_t id){

	pthread_cancel(id);
	pthread_join(id,NULL);


}
