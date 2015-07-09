//============================================================================
// Name        : template.cpp
// Author      : Chiu
// Description : Robotiq Adaptive 3 Finger Template
//============================================================================

#include "Gripper.h"
#include <pthread.h>
#define sec 1000000 //!<sec in microseconds
#define ms  1000    //!<ms in microseconds

//!thread purposes
enum Thread {Com, Fault};
pthread_t launch(Gripper*, Thread);  //!<thread launcher
void terminate(pthread_t);	 //!<thread terminator

//============================================================================
////////////////////////////////start of main//////////////////////////////////
//============================================================================



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
	while(!gripper.isActivated()){

		gripper.activate();
		usleep(sec);

	}cout << "activated" <<endl<<endl;


//============================================================================
// gripper do stuff here
// refer to the interface documentation interface.pdf for available methods
//============================================================================


	//!Deactivate
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


//============================================================================
////////////////////////////////end of main//////////////////////////////////
//============================================================================



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

