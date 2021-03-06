// ===================================================================
// Wiimote external for Puredata
// Written by Mike Wozniewki (Feb 2007), www.mikewoz.com
//
// Requires the CWiid library (version 0.6.00) by L. Donnie Smith
//
// ===================================================================
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
// ===================================================================

//  ChangeLog:
//  2008-04-14 Florian Krebs 
//  * adapt wiimote external for the actual version of cwiid (0.6.00)
//  2009-11-05 Robert Künnemann
//  * code clean ups, depencency on cwiid_internal removed

#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <bluetooth/bluetooth.h>
#include <m_pd.h>
#include <math.h>
#include <cwiid.h>
//#include <cwiid_internal.h> //get rid of this, if possible

#define PI	3.14159265358979323

struct acc {
	unsigned char x;
	unsigned char y;
	unsigned char z;
};

/* Wiimote Callback */
cwiid_mesg_callback_t cwiid_callback;

// class and struct declarations for wiimote pd external:
static t_class *cwiid_class;
typedef struct _wiimote
{
	t_object x_obj; // standard pd object (must be first in struct)
	
	cwiid_wiimote_t *wiimote; // individual wiimote handle per pd object, represented in libcwiid

	t_float connected;
	int listID;
	
	t_float toggle_acc, toggle_ir, toggle_nc;

	struct acc acc_zero, acc_one; // acceleration
	struct acc nc_acc_zero, nc_acc_one; // nunchuck acceleration

	// We store atom list for each data type so we don't waste time
	// allocating memory at every callback:
	t_atom btn_atoms[2];
	t_atom acc_atoms[3];
	t_atom ir_atoms[4];
	t_atom nc_btn_atoms[2];
	t_atom nc_acc_atoms[3];
	t_atom nc_stick_atoms[2];
	
	// outlets:
	t_outlet *outlet_btn;
	t_outlet *outlet_acc;
	t_outlet *outlet_ir;
	t_outlet *outlet_nc_btn;
	t_outlet *outlet_nc_acc;
	t_outlet *outlet_nc_stick;
	
} t_wiimote;

#define MAX_WIIMOTES 8
t_wiimote *g_wiimoteList[MAX_WIIMOTES];


// ==============================================================
void cwiid_debug(t_wiimote *x)
{
	post("\n======================");
	if (x->connected) post("Wiimote (id: %d) is connected.", x->listID);
	else post("Wiimote (id: %d) is NOT connected.", x->listID);
	if (x->toggle_acc) post("acceleration: ON");
	else post("acceleration: OFF");
	if (x->toggle_ir)  post("IR:           ON");
	else post("IR:           OFF");
	if (x->toggle_nc)  post("Nunchuck:     ON");
	else post("Nunchuck:     OFF");
	post("");
	post("Accelerometer calibration: zero=(%d,%d,%d) one=(%d,%d,%d)",x->acc_zero.x,x->acc_zero.y,x->acc_zero.z,x->acc_one.x,x->acc_one.y,x->acc_one.z);
	post("Nunchuck calibration:      zero=(%d,%d,%d) one=(%d,%d,%d)",x->nc_acc_zero.x,x->nc_acc_zero.y,x->nc_acc_zero.z,x->nc_acc_one.x,x->nc_acc_one.y,x->nc_acc_one.z);
	

}

// ==============================================================

// Button handler:
void cwiid_btn(t_wiimote *x, struct cwiid_btn_mesg *mesg)
{
	//post("Buttons: %X %X", (mesg->buttons & 0xFF00)>>8, data->btn_data.buttons & 0x00FF);
	SETFLOAT(x->btn_atoms+0, (mesg->buttons & 0xFF00)>>8);
	SETFLOAT(x->btn_atoms+1, mesg->buttons & 0x00FF);
	outlet_anything(x->outlet_btn, &s_list, 2, x->btn_atoms);
/*
	if (mesg->buttons & CWIID_BTN_UP) {}
	if (mesg->buttons & CWIID_BTN_DOWN) {}
	if (mesg->buttons & CWIID_BTN_LEFT) {}
	if (mesg->buttons & CWIID_BTN_RIGHT) {}
	if (mesg->buttons & CWIID_BTN_A) {}
	if (mesg->buttons & CWIID_BTN_B) {}
	if (mesg->buttons & CWIID_BTN_MINUS) {}
	if (mesg->buttons & CWIID_BTN_PLUS) {}
	if (mesg->buttons & CWIID_BTN_HOME) {}
	if (mesg->buttons & CWIID_BTN_1) {}
	if (mesg->buttons & CWIID_BTN_2) {}
*/
	
}


void cwiid_acc(t_wiimote *x, struct cwiid_acc_mesg *mesg)
{
	if (x->toggle_acc)
	{
		double a_x, a_y, a_z;
		
		a_x = ((double)mesg->acc[CWIID_X] - x->acc_zero.x) / (x->acc_one.x - x->acc_zero.x);
		a_y = ((double)mesg->acc[CWIID_Y] - x->acc_zero.y) / (x->acc_one.y - x->acc_zero.y);
		a_z = ((double)mesg->acc[CWIID_Z] - x->acc_zero.z) / (x->acc_one.z - x->acc_zero.z);
		
		/*
		double a, roll, pitch;
		a = sqrt(pow(a_x,2)+pow(a_y,2)+pow(a_z,2));
		roll = atan(a_x/a_z);
		if (a_z <= 0.0) roll += PI * ((a_x > 0.0) ? 1 : -1);
		roll *= -1;
		pitch = atan(a_y/a_z*cos(roll));
		*/
		
		SETFLOAT(x->acc_atoms+0, a_x);
		SETFLOAT(x->acc_atoms+1, a_y);
		SETFLOAT(x->acc_atoms+2, a_z);
		outlet_anything(x->outlet_acc, &s_list, 3, x->acc_atoms);
	}
	
}

void cwiid_ir(t_wiimote *x, struct cwiid_ir_mesg *mesg)
{
	unsigned int i;

	if (x->toggle_ir)
	{
		//post("IR (valid,x,y,size) #%d: %d %d %d %d", i, data->ir_data.ir_src[i].valid, data->ir_data.ir_src[i].x, data->ir_data.ir_src[i].y, data->ir_data.ir_src[i].size);
		for (i=0; i<CWIID_IR_SRC_COUNT; i++)
		{		
			if (mesg->src[i].valid)
			{
				SETFLOAT(x->ir_atoms+0, i);
				SETFLOAT(x->ir_atoms+1, mesg->src[i].pos[CWIID_X]);
				SETFLOAT(x->ir_atoms+2, mesg->src[i].pos[CWIID_Y]);
				SETFLOAT(x->ir_atoms+3, mesg->src[i].size);
				outlet_anything(x->outlet_ir, &s_list, 4, x->ir_atoms);
			}
		}
	}
}


void cwiid_nunchuk(t_wiimote *x, struct cwiid_nunchuk_mesg *mesg)
{
	double a_x, a_y, a_z;

	a_x = ((double)mesg->acc[CWIID_X] - x->nc_acc_zero.x) / (x->nc_acc_one.x - x->nc_acc_zero.x);
	a_y = ((double)mesg->acc[CWIID_Y] - x->nc_acc_zero.y) / (x->nc_acc_one.y - x->nc_acc_zero.y);
	a_z = ((double)mesg->acc[CWIID_Z] - x->nc_acc_zero.z) / (x->nc_acc_one.z - x->nc_acc_zero.z);

	/*
	double a, roll, pitch;
	a = sqrt(pow(a_x,2)+pow(a_y,2)+pow(a_z,2));
	roll = atan(a_x/a_z);
	if (a_z <= 0.0) roll += PI * ((a_x > 0.0) ? 1 : -1);
	roll *= -1;
	pitch = atan(a_y/a_z*cos(roll));
	*/
	
	if (mesg->buttons & CWIID_NUNCHUK_BTN_C) {}
	if (mesg->buttons & CWIID_NUNCHUK_BTN_Z) {}
	outlet_float(x->outlet_nc_btn, mesg->buttons);
	
	SETFLOAT(x->nc_acc_atoms+0, a_x);
	SETFLOAT(x->nc_acc_atoms+1, a_y);
	SETFLOAT(x->nc_acc_atoms+2, a_z);
	outlet_anything(x->outlet_nc_acc, &s_list, 3, x->nc_acc_atoms);
	
	SETFLOAT(x->nc_stick_atoms+0, mesg->stick[CWIID_X]);
	SETFLOAT(x->nc_stick_atoms+1, mesg->stick[CWIID_Y]);
	outlet_anything(x->outlet_nc_stick, &s_list, 2, x->nc_stick_atoms);
	
}

// The CWiid library invokes a callback function whenever events are
// generated by the wiimote. This function is specified when connecting
// to the wiimote (in the cwiid_open function).

// Unfortunately, the mesg struct passed as an argument to the
// callback does not have a pointer to the wiimote instance, and it
// is thus impossible to know which wiimote has invoked the callback.
// For this case we provide a hard-coded set of wrapper callbacks to
// indicate which Pd wiimote instance to control.

// So far I have only checked with one wiimote

/*void cwiid_callback(cwiid_wiimote_t *wiimt, int mesg_count, union cwiid_mesg *mesg[], struct timespec *timestamp)
*/

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
                    union cwiid_mesg mesg_array[], struct timespec *timestamp)
{
	unsigned char buf[7];
	int i;
	t_wiimote *x;
	
	int wiimote_id=0;

	wiimote_id = cwiid_get_id(wiimote);

	if (g_wiimoteList[wiimote_id] == NULL) {
		post("no wiimote loaded: %d%",wiimote_id);
	}
	else {	
		x = g_wiimoteList[wiimote_id];
			
		for (i=0; i < mesg_count; i++)
		{	
			switch (mesg_array[i].type) {
				case CWIID_MESG_STATUS:
					post("Battery: %d%", (int) (100.0 * mesg_array[i].status_mesg.battery / CWIID_BATTERY_MAX));
					switch (mesg_array[i].status_mesg.ext_type) {
						case CWIID_EXT_NONE:
							post("No nunchuck attached");
							break;
						case CWIID_EXT_NUNCHUK:
							post("Nunchuck extension attached");
						
							if (cwiid_read(x->wiimote, CWIID_RW_REG | CWIID_RW_DECODE, 0xA40020, 								7, buf)) {
								post("Unable to retrieve Nunchuk calibration");
							}
							else {
								x->nc_acc_zero.x = buf[0];
								x->nc_acc_zero.y = buf[1];
								x->nc_acc_zero.z = buf[2];
								x->nc_acc_one.x  = buf[4];
								x->nc_acc_one.y  = buf[5];
								x->nc_acc_one.z  = buf[6];
							}	
							break;
						case CWIID_EXT_CLASSIC:
							post("Classic controller attached. There is no support for this yet.");
							break;
						case CWIID_EXT_UNKNOWN:
							post("Unknown extension attached");
							break;
					}
					break;
				case CWIID_MESG_BTN:
					cwiid_btn(x, &mesg_array[i].btn_mesg);
					break;
				case CWIID_MESG_ACC:
					cwiid_acc(x, &mesg_array[i].acc_mesg);
					break;
				case CWIID_MESG_IR:
					cwiid_ir(x, &mesg_array[i].ir_mesg);
					break;
				case CWIID_MESG_NUNCHUK:
					cwiid_nunchuk(x, &mesg_array[i].nunchuk_mesg);
					break;
				case CWIID_MESG_CLASSIC:
					// todo
					break;
				default:
					break;
			}
		}
	}
}
/*
void cwiid_callback_0(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[0], 0,  mesg, 0); }
void cwiid_callback_1(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[1], mesg); }
void cwiid_callback_2(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[2], mesg); }
void cwiid_callback_3(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[3], mesg); }
void cwiid_callback_4(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[4], mesg); }
void cwiid_callback_5(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[5], mesg); }
void cwiid_callback_6(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[6], mesg); }
void cwiid_callback_7(union cwiid_mesg *mesg) { cwiid_callback(g_wiimoteList[7], mesg); }
*/

// ==============================================================



void cwiid_setReportMode(t_wiimote *x, t_floatarg r)
{
	unsigned char rpt_mode;

	if (r >= 0) rpt_mode = (unsigned char) r;
	else {
		rpt_mode = CWIID_RPT_STATUS | CWIID_RPT_BTN;
		if (x->toggle_ir) rpt_mode |= CWIID_RPT_IR;
		if (x->toggle_acc) rpt_mode |= CWIID_RPT_ACC;
		if (x->toggle_nc) rpt_mode |= CWIID_RPT_EXT;
	}
	if (x->connected)
	{
		if (cwiid_command(x->wiimote, CWIID_CMD_RPT_MODE, rpt_mode)) {
			post("wiimote error: problem setting report mode.");
		}
	}
}

void cwiid_reportAcceleration(t_wiimote *x, t_floatarg f)
{
	x->toggle_acc = 1;
	cwiid_setReportMode(x, -1);
}

void cwiid_reportIR(t_wiimote *x, t_floatarg f)
{
	x->toggle_ir = 1;
	cwiid_setReportMode(x, -1);
}

void cwiid_reportNunchuck(t_wiimote *x, t_floatarg f)
{
	x->toggle_nc = 1;
	cwiid_setReportMode(x, -1);
}

void cwiid_setRumble(t_wiimote *x, t_floatarg f)
{
	if (x->connected)
	{
		if (cwiid_command(x->wiimote, CWIID_CMD_RUMBLE, f)) post("wiiremote error: problem setting rumble.");
	}
}

void cwiid_setLED(t_wiimote *x, t_floatarg f)
{
	// some possible values:
	// CWIID_LED0_ON		0x01
	// CWIID_LED1_ON		0x02
	// CWIID_LED2_ON		0x04
	// CWIID_LED3_ON		0x08
	if (x->connected)
	{
		if (cwiid_command(x->wiimote, CWIID_CMD_LED, f)) post("wiiremote error: problem setting LED.");
	}
}






// ==============================================================


// The following function attempts to connect to a wiimote at a
// specific address, provided as an argument. eg, 00:19:1D:70:CE:72
// This address can be discovered by running the following command
// in a console:
//   hcitool scan | grep Nintendo

void cwiid_doConnect(t_wiimote *x, t_symbol *addr)
{
	unsigned char buf[7];
	bdaddr_t bdaddr;
	// determine address:
	if (addr==gensym("NULL")) {
		post("Searching automatically...");		
		bdaddr = *BDADDR_ANY;
	}
	else {
		str2ba(addr->s_name, &bdaddr);
		post("Connecting to given address...");
		post("Press buttons 1 and 2 simultaneously.");
		} 

	// connect:
	
	//search for next free item in list
	int i;
	for(i=0; i < MAX_WIIMOTES; i++) 
	{
		if (g_wiimoteList[i]==NULL) {
			post("open: Connect wiimote %d", i);		
			x->wiimote = cwiid_open(&bdaddr,CWIID_FLAG_MESG_IFC);
			x->listID = i;
			g_wiimoteList[i] = x;
			break;
		}
		else if( i == MAX_WIIMOTES -1)
		{
			post("Error: The maximum number of wiimotes has been reached.");
			return;
		}
	}

	if (x->wiimote == NULL) {
		post("wiimote error: unable to connect");
	} else {
		post("wiimote has successfully connected");
		if (cwiid_read(x->wiimote, CWIID_RW_EEPROM, 0x16, 7, buf)) {
			post("Unable to retrieve accelerometer calibration");
		} else {
			x->acc_zero.x = buf[0];
			x->acc_zero.y = buf[1];
			x->acc_zero.z = buf[2];
			x->acc_one.x  = buf[4];
			x->acc_one.y  = buf[5];
			x->acc_one.z  = buf[6];
			//post("Retrieved wiimote calibration: zero=(%.1f,%.1f,%.1f) one=(%.1f,%.1f,%.1f)",buf[0],buf[2],buf[3],buf[4],buf[5],buf[6]);
		}
		x->connected = 1;
		cwiid_setReportMode(x,-1);
		if (cwiid_set_mesg_callback(x->wiimote, &cwiid_callback)) {
			fprintf(stderr, "Unable to set message callback\n");
		}
	}
}

// The following function attempts to discover a wiimote. It requires
// that the user puts the wiimote into 'discoverable' mode before being
// called. This is done by pressing the red button under the battery
// cover, or by pressing buttons 1 and 2 simultaneously.
// TODO: Without pressing the buttons, I get a segmentation error. So far, I don't know why.

void cwiid_discover(t_wiimote *x)
{
	post("Put the wiimote into discover mode by pressing buttons 1 and 2 simultaneously.");

	cwiid_doConnect(x, gensym("NULL"));
	if (!(x->connected))
	{
		post("Error: could not find any wiimotes. Please ensure that bluetooth is enabled, and that the 		'hcitool scan' command lists your Nintendo device.");
	}
}

void cwiid_doDisconnect(t_wiimote *x)
{

	if (x->connected)
	{
		if (cwiid_close(x->wiimote)) {
			post("wiimote error: problems when disconnecting.");
		} 
		else {
			post("disconnect successfull, resetting values");
			g_wiimoteList[x->listID] = NULL;
			x->connected = 0;
		}
	}
	else post("device is not connected");

}


// ==============================================================
// ==============================================================

static void *cwiid_new(t_symbol* s, int argc, t_atom *argv)
{
	bdaddr_t bdaddr; // wiimote bdaddr
	t_wiimote *x = (t_wiimote *)pd_new(cwiid_class);

	// create outlets:
	x->outlet_btn = outlet_new(&x->x_obj, &s_list);
	x->outlet_acc = outlet_new(&x->x_obj, &s_list);
	x->outlet_ir = outlet_new(&x->x_obj, &s_list);
	x->outlet_nc_btn = outlet_new(&x->x_obj, &s_float);
	x->outlet_nc_acc = outlet_new(&x->x_obj, &s_list);
	x->outlet_nc_stick = outlet_new(&x->x_obj, &s_list);

	// initialize toggles:
	x->toggle_acc = 0;
	x->toggle_ir = 0;
	x->toggle_nc = 0;

	x->connected = 0;
	x->listID = -1;

	// connect if user provided an address as an argument:

	if (argc==2)
	{
		post("conecting to provided address...");
		if (argv->a_type == A_SYMBOL)
		{
			cwiid_doConnect(x, atom_getsymbol(argv));
		} else {
			error("[wiimote] expects either no argument, or a bluetooth address as an argument. eg, 00:19:1D:70:CE:72");
			return NULL;
		}
	}
	return (x);
}


static void cwiid_free(t_wiimote* x)
{
	cwiid_doDisconnect(x);
}

void wiimote_setup(void)
{
	int i;
	for (i=0; i<MAX_WIIMOTES; i++) g_wiimoteList[i] = NULL;

	cwiid_class = class_new(gensym("wiimote"), (t_newmethod)cwiid_new, (t_method)cwiid_free, sizeof(t_wiimote), CLASS_DEFAULT, A_GIMME, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_debug, gensym("debug"), 0);
	class_addmethod(cwiid_class, (t_method) cwiid_doConnect, gensym("connect"), A_SYMBOL, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_doDisconnect, gensym("disconnect"), 0);
	class_addmethod(cwiid_class, (t_method) cwiid_discover, gensym("discover"), 0);
	class_addmethod(cwiid_class, (t_method) cwiid_setReportMode, gensym("setReportMode"), A_DEFFLOAT, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_reportAcceleration, gensym("reportAcceleration"), A_DEFFLOAT, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_reportNunchuck, gensym("reportNunchuck"), A_DEFFLOAT, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_reportIR, gensym("reportIR"), A_DEFFLOAT, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_setRumble, gensym("setRumble"), A_DEFFLOAT, 0);
	class_addmethod(cwiid_class, (t_method) cwiid_setLED, gensym("setLED"), A_DEFFLOAT, 0);
}


