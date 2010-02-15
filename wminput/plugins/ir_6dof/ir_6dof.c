/* Copyright (C) 2007 L. Donnie Smith <donnie.smith@gatech.edu>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <math.h>

#include "wmplugin.h"

#define PI	3.14159265358979323

static unsigned char info_init = 0;
static struct wmplugin_info info;
static struct wmplugin_data data;

static struct acc_cal acc_cal;

static int plugin_id;

wmplugin_info_t wmplugin_info;
wmplugin_init_t wmplugin_init;
wmplugin_exec_t wmplugin_exec;
static void process_acc(struct cwiid_acc_mesg *mesg);

static float Yaw_Scale = 1.0;
static float Roll_Scale = 1.0;
static float Pitch_Scale = 1.0;
static float X_Scale = 1.0;
static float Y_Scale = 1.0;
static float Z_Scale = 1.0;

struct wmplugin_info *wmplugin_info() {
	if (!info_init) {
		info.button_count = 0;
        info.axis_count = 6;
        info.axis_info[0].name = "X";
        info.axis_info[0].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[0].max  = 16;
        info.axis_info[0].min  = -16;
        info.axis_info[0].fuzz = 0;
        info.axis_info[0].flat = 0;
        info.axis_info[1].name = "Y";
        info.axis_info[1].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[1].max  = 16;
        info.axis_info[1].min  = -16;
        info.axis_info[1].fuzz = 0;
        info.axis_info[1].flat = 0;
        info.axis_info[2].name = "Z";
        info.axis_info[2].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[2].max  = 50;
        info.axis_info[2].min  = 16;
        info.axis_info[2].fuzz = -16;
        info.axis_info[2].flat = 0;
        info.axis_info[3].name = "Roll";
        info.axis_info[3].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[3].max  = 16;
        info.axis_info[3].min  = -16;
        info.axis_info[3].fuzz = 0;
        info.axis_info[3].flat = 0;
        info.axis_info[4].name = "Yaw";
        info.axis_info[4].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[4].max  = 16;
        info.axis_info[4].min  = -16;
        info.axis_info[4].fuzz = 0;
        info.axis_info[4].flat = 0;
        info.axis_info[5].name = "Pitch";
        info.axis_info[5].type = WMPLUGIN_ABS | WMPLUGIN_REL;
        info.axis_info[5].max  = 16;
        info.axis_info[5].min  = -16;
        info.axis_info[5].fuzz = 0;
        info.axis_info[5].flat = 0;
        info.param_count = 4;
        info.param_info[0].name = "X_Scale";
        info.param_info[0].type = WMPLUGIN_PARAM_FLOAT;
        info.param_info[0].ptr = &X_Scale;
        info.param_info[1].name = "Y_Scale";
        info.param_info[1].type = WMPLUGIN_PARAM_FLOAT;
        info.param_info[1].ptr = &Y_Scale;

		info_init = 1;
	}
	return &info;
}

int wmplugin_init(int id, cwiid_wiimote_t *wiimote)
{
	plugin_id = id;

	data.buttons = 0;
	data.axes[0].valid = 1;
	data.axes[1].valid = 1;
	if (wmplugin_set_rpt_mode(id, CWIID_RPT_ACC)) {
		return -1;
	}

	if (cwiid_get_acc_cal(wiimote, CWIID_EXT_NONE, &acc_cal)) {
		wmplugin_err(id, "calibration error");
		return -1;
	}

	return 0;
}

struct wmplugin_data *wmplugin_exec(int mesg_count, union cwiid_mesg mesg[])
{
	int i;
	struct wmplugin_data *ret = NULL;

	for (i=0; i < mesg_count; i++) {
		switch (mesg[i].type) {
		case CWIID_MESG_ACC:
			process_acc(&mesg[i].acc_mesg);
			ret = &data;
			break;
		default:
			break;
		}
	}

	return ret;
}

#define NEW_AMOUNT 0.1
#define OLD_AMOUNT (1.0-NEW_AMOUNT)
double a_x = 0, a_y = 0, a_z = 0;

static void process_acc(struct cwiid_acc_mesg *mesg)
{
	double a;
	double roll, pitch;

	a_x = (((double)mesg->acc[CWIID_X] - acc_cal.zero[CWIID_X]) /
	      (acc_cal.one[CWIID_X] - acc_cal.zero[CWIID_X]))*NEW_AMOUNT +
	      a_x*OLD_AMOUNT;
	a_y = (((double)mesg->acc[CWIID_Y] - acc_cal.zero[CWIID_Y]) /
	      (acc_cal.one[CWIID_Y] - acc_cal.zero[CWIID_Y]))*NEW_AMOUNT +
	      a_y*OLD_AMOUNT;
	a_z = (((double)mesg->acc[CWIID_Z] - acc_cal.zero[CWIID_Z]) /
	      (acc_cal.one[CWIID_Z] - acc_cal.zero[CWIID_Z]))*NEW_AMOUNT +
	      a_z*OLD_AMOUNT;

	a = sqrt(pow(a_x,2)+pow(a_y,2)+pow(a_z,2));
	roll = atan(a_x/a_z);
	if (a_z <= 0.0) {
		roll += PI * ((a_x > 0.0) ? 1 : -1);
	}

	pitch = atan(a_y/a_z*cos(roll));

	data.axes[3].value = roll  * 1000 * Roll_Scale;
	data.axes[4].value = pitch * 1000 * Pitch_Scale;

	if ((a > 0.85) && (a < 1.15)) {
		if ((fabs(roll)*(180/PI) > 10) && (fabs(pitch)*(180/PI) < 80)) {
			data.axes[0].valid = 1;
			data.axes[0].value = roll * 5 * X_Scale;
		}
		else {
			data.axes[0].valid = 0;
		}
		if (fabs(pitch)*(180/PI) > 10) {
			data.axes[1].valid = 1;
			data.axes[1].value = pitch * 10 * Y_Scale;
		}
		else {
			data.axes[1].valid = 0;
		}
	}
	else {
		data.axes[0].valid = 0;
		data.axes[1].valid = 0;
	}
}

