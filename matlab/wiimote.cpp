#include <string>
#include <ctime>
#include <list>

#include <stdio.h>

#include <cwiid.h>
#include "mex.h"

#include "wiimote-utils.cpp"


/*#include <fstream.h>
#include <iostream.h> */

/* 
 *
 * Input: either: bluetooth-adress of wiimote
 * 	or:	empty for auto-discover
 * Output:	
 * 		
 * 		*/

cwiid_wiimote_t *wiimote=NULL;	/* wiimote handle */

time_t buf_connect_time=0; /*notes down when capture began for get_buf */

time_t connect_time; /*It's capture time, baby */

std::list<struct dataset> q;

pthread_mutex_t q_lock=  PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t q_pop_lock=  PTHREAD_MUTEX_INITIALIZER;

cwiid_mesg_callback_t cwiid_callback;

mxArray* copy_into_matrix(std::list<struct dataset> & queue , time_t capture_time)
{
	double *outArray; /*pointer to data for output */

	int rows = queue.size();
	mxArray * matrix = mxCreateDoubleMatrix(rows , 8,mxREAL);
	outArray = mxGetPr(matrix);

	for(int i=0; i<rows; i++)
	{
		struct dataset data = queue.front(); 
		outArray[i+0*rows] = compute_relative_time(data.timestamp, capture_time);
		for(int j=0; j<3; j++)
			outArray[i+(j+1)*rows] = data.acc[j];
		for(int j=0; j<3; j++)
			outArray[i+(j+4)*rows] = data.rate[j];
		outArray[i+(7)*rows] = data.button;

		queue.pop_front();
	}

	return matrix;
}

void connect(bdaddr_t bdaddr)
{
	unsigned char rpt_mode = CWIID_RPT_ACC | CWIID_RPT_EXT | CWIID_RPT_BTN;

	if(wiimote == NULL) /*not initialized yet */
	{

		if (!(wiimote = cwiid_open(&bdaddr, 0))) {
			mexErrMsgTxt("Unable to connect to wiimote.");
			return;
		}

		if(cwiid_enable(wiimote, CWIID_FLAG_MOTIONPLUS))
		{
			mexErrMsgTxt("Error setting MotionPlus mode.");
			return;
		}
		if (cwiid_set_rpt_mode(wiimote, rpt_mode)) {
			mexErrMsgTxt("Error setting report mode.");
			return;
		}

		if(cwiid_enable(wiimote, CWIID_FLAG_MESG_IFC | CWIID_FLAG_REPEAT_BTN))
		{
			mexErrMsgTxt("Error Putting into message mode.");
			return;
		}
		time(&connect_time);

		cwiid_set_mesg_callback(wiimote, cwiid_callback);

		pthread_mutex_init(&q_lock,NULL);
		pthread_mutex_init(&q_pop_lock,NULL);
	}
	return;
}

int disconnect()
{
	if(wiimote != NULL) /*only if initialized */
	{

		if (cwiid_close(wiimote)) {
			mexErrMsgTxt("Unable to disconnect wiimote.");
			return -1;
		}
		q = std::list<dataset>(); //clear queue

		pthread_mutex_destroy(&q_lock);
		pthread_mutex_destroy(&q_pop_lock);
	}
	return 0;
}

void capture(const std::list<struct dataset> & from_q, std::list<struct dataset> & to_q, time_t capture_time, uint16_t button)
{
	bool capture_started=false;
	bool capture_finished=false;


	std::list<struct dataset>::const_iterator i = from_q.begin();
	
	if(i != from_q.end()) //queue not empty
	{
		i = from_q.end();
		i--; //go to position of last (newest) element, 
	}	

	//mit pop_lock vor invalid werden des iterators schuetzen
	pthread_mutex_lock(&q_pop_lock);

	std::list<struct dataset>::const_iterator j;
	while(not capture_finished)
	{
		j=i; j++;
		if(j == from_q.end()){
			fprintf(stderr, "Empty queue\n");
			usleep(0.1e+6); // sleep for one tenth of a second
		}	
		else
		{
			struct dataset data = *j;
			if(! capture_started)
			{
				fprintf(stderr, "Not capturing yet..%x, %x \n", data.button, button);
				if((data.button & button) == button)
				{ 
					capture_started=true;
					to_q.push_back(data);
				}
			}
			else
			{ //capture_started
				fprintf(stderr, "Capturing.%x, %x \n", data.button, button);
				if((data.button & button) != button)
				{
					capture_finished=true;
				}
				else
				{
					to_q.push_back(data);
				}
			}
		}
		i++;
	}
	pthread_mutex_unlock(&q_pop_lock);
}

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
		union cwiid_mesg mesg[], struct timespec *timestamp)
{
	int i;

	struct dataset data;

	data.timestamp=*timestamp;
	data.button = 0;

	//do mutex stuff	
	for (i=0; i < mesg_count; i++)
	{
		switch (mesg[i].type) {
			case CWIID_MESG_ACC:
				for(int j=0; j<3; j++)
					data.acc[j]=mesg[i].acc_mesg.acc[j];
				break;
			case CWIID_MESG_MOTIONPLUS:
				for(int j=0; j<3; j++)
					data.rate[j]= mesg[i].motionplus_mesg.angle_rate[j];
				break;
			case CWIID_MESG_ERROR:
				if (cwiid_close(wiimote)) {
					mexErrMsgTxt("Error on wiimote disconnect\n");
					exit(-1);
				}
				exit(0);
				break;
			case CWIID_MESG_BTN:
				data.button= mesg[i].btn_mesg.buttons;
				/*if(data.button != 0)
				  fprintf(stderr, "Reporting Button %x \n", data.button);*/
				break;
			default:
				printf("Unknown Report");
				break;
		}
	}
	pthread_mutex_lock(&q_lock);
	q.push_back(data);
	pthread_mutex_unlock(&q_lock);
}


void mexFunction(int nlhs, mxArray *plhs[], 
		int nrhs, const mxArray *prhs[])
{
	bdaddr_t bdaddr;	/* bluetooth device address */
	int buflen;
	char* command=NULL;

	if(nrhs > 0)
	{
		/* input must be a string */
		if ( mxIsChar(prhs[0]) != 1)
			mexErrMsgTxt("Input must be a string.");
		/* input must be a row vector */
		if (mxGetM(prhs[0])!=1)
			mexErrMsgTxt("Input must be a row vector.");
		/* get the length of the input string */
		buflen = (mxGetM(prhs[0]) * mxGetN(prhs[0])) + 1;
		/* copy the string data from prhs[0] into a C string input_ buf.    */
		command = mxArrayToString(prhs[0]);

		if(strcmp(command, "connect" )==0)
		{
			if(nrhs == 2)
			{
				/*TODO convert string into bluetooth adress.. */
			}
			else if (nrhs == 1) /* if none is given */
			{
				bdaddr = *BDADDR_ANY; 
			}
			else
			{
				mexErrMsgTxt("Too many arguments.");
			}

			connect(bdaddr);
		}
		else if (strcmp(command, "disconnect" )==0)
		{
			disconnect();
		}
		else if (strcmp(command, "snapshot" )==0)
		{
			if(nlhs != 1)
			{
				mexErrMsgTxt("One output argument needed.");
			}
			if(nrhs != 1)
			{
				mexErrMsgTxt("Need no argument.");
			}

			//int stream_buffer_length = mxGetScalar(prhs[1]);

			pthread_mutex_lock(&q_pop_lock);
			pthread_mutex_lock(&q_lock);
			plhs[0] = copy_into_matrix(q, connect_time);
			pthread_mutex_unlock(&q_lock);
			pthread_mutex_unlock(&q_pop_lock);
		}
		else if (strcmp(command, "capture" )==0)
		{
			if(nlhs != 1)
			{
				mexErrMsgTxt("One output argument needed.");
			}
			if(nrhs != 1)
			{
				mexErrMsgTxt("Need no argument.");
			}

			time_t capture_time;
			time(&capture_time);

			std::list<struct dataset> capture_queue;
			capture(q, capture_queue, capture_time, 0x04);	//use button Z for capture...
			plhs[0] = copy_into_matrix(capture_queue, capture_time);
		}

		else{
			mexErrMsgTxt("Unknown Command.");
		}
	}
	else{
		mexErrMsgTxt("Command missing.");
	}
}
