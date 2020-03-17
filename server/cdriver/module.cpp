/* module.cpp - Python glue for Franklin
 * Copyright 2018 Bas Wijnen <wijnen@debian.org>
 * Author: Bas Wijnen <wijnen@debian.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <sys/mman.h>
#include <unistd.h>
#include <libgen.h>
#include "module.h"
#include <linux/memfd.h>
#include <sys/syscall.h>
#include <poll.h>
#include <cmath>

#ifndef memfd_create
#define memfd_create(...) syscall(SYS_memfd_create, __VA_ARGS__)
#endif

#if 0
#define FUNCTION_START do { debug("Entering %s:%d", __PRETTY_FUNCTION__, __LINE__); } while (0)
#define DEBUG_FUNCTIONS true
#else
#define FUNCTION_START do {} while (0)
#define DEBUG_FUNCTIONS false
#endif

// Debug communication.
//#define cdebug debug
#define cdebug(...) do {} while (0)

static void print_dict(PyObject *dict) {
	PyObject *keys = PyDict_Keys(dict);
	for (Py_ssize_t i = 0; i < PyList_Size(keys); ++i) {
		PyObject *key = PyList_GetItem(keys, i);
		debug("\t%s", PyUnicode_AsUTF8(key));
	}
	Py_DECREF(keys);
}

static PyObject *assert_empty_dict(PyObject *dict, char const *src) {
	if (PyDict_Size(dict) != 0) {
		debug("Warning: dict for %s contained unused keys:", src);
		print_dict(dict);
	}
	Py_DECREF(dict);
	Py_RETURN_NONE;
}

static inline char send_to_child(char cmd) {
	cdebug("sending to child: %x", cmd);
	struct pollfd test;
	test.fd = toserver;
	test.events = POLLIN | POLLPRI;
	test.revents = 0;
	int p = poll(&test, 1, 0);
	if (p < 0) {
		debug("send_to_child: test poll fails: %s", strerror(errno));
	}
	else if (p > 0) {
		debug("send_to_child: data was ready before request was sent");
		exit(1);
	}
	if (cmd != (char)-1) {
		cdebug("actually writing");
		while (true) {
			errno = 0;
			int ret = write(fromserver, &cmd, 1);
			if (ret == 1)
				break;
			if (errno == EINTR || errno == EAGAIN)
				continue;
			debug("failed to send %x to child: %s (%d)", cmd, strerror(errno), errno);
			exit(0);
		}
	}
	cdebug("done writing; reading");
	while (true) {
		errno = 0;
		int ret = read(toserver, &cmd, 1);
		if (ret == 1)
			break;
		if (errno == EINTR || errno == EAGAIN)
			continue;
		debug("failed to receive reply to %x from child: %s (%d)", cmd, strerror(errno), errno);
		exit(0);
	}
	cdebug("read %x", cmd);
	return cmd;
}

#ifdef SERIAL
static PyObject *set_uuid(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	const char *new_uuid;
	Py_ssize_t length;
	if (!PyArg_ParseTuple(args, "y#", &new_uuid, &length))
		return NULL;
	if (length != UUID_SIZE)
		return PyErr_Format(PyExc_ValueError, "uuid must be {} bytes long", UUID_SIZE);
	for (int i = 0; i < UUID_SIZE; ++i) {
		shmem->uuid[i] = new_uuid[i];
		debug("uuid %x: %02x", i, shmem->uuid[i]);
	}
	send_to_child(CMD_SET_UUID);
	Py_RETURN_NONE;
}

static PyObject *get_uuid(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// get uuid as received from firmware.
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	return PyBytes_FromStringAndSize(reinterpret_cast <const char *>(const_cast <const unsigned char *>(shmem->uuid)), UUID_SIZE);
}

static PyObject *force_disconnect(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_FORCE_DISCONNECT);
	Py_RETURN_NONE;
}

static PyObject *connect_machine(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	char *id, *port;
	if (!PyArg_ParseTuple(args, "y#y", &id, &shmem->ints[0], &port))
		return NULL;
	memcpy(const_cast <char *>(shmem->strs[0]), id, ID_SIZE);
	strncpy(const_cast <char *>(shmem->strs[1]), port, PATH_MAX);
	send_to_child(CMD_CONNECT);
	Py_RETURN_NONE;
}

static PyObject *reconnect(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	char *port;
	if (!PyArg_ParseTuple(args, "y", &port))
		return NULL;
	strncpy(const_cast <char *>(shmem->strs[0]), port, PATH_MAX);
	send_to_child(CMD_RECONNECT);
	Py_RETURN_NONE;
}
#endif

static PyObject *move(PyObject *Py_UNUSED(self), PyObject *args, PyObject *keywords) {
	FUNCTION_START;
	shmem->move.single = false;
	shmem->move.probe = false;
	shmem->move.pattern_size = 0;
	shmem->ints[0] = 0;
	const char *keywordnames[] = {"tool", "x", "y", "z", "a", "b", "c", "e", "v", "single", "probe", "relative", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, keywords, "idddddddd|ppp", const_cast <char **>(keywordnames),
				&shmem->move.tool,
				&shmem->move.target[0], &shmem->move.target[1], &shmem->move.target[2],
				&shmem->move.abc[0], &shmem->move.abc[1], &shmem->move.abc[2],
				&shmem->move.e,
				&shmem->move.v0,
				&shmem->move.single,
				&shmem->move.probe,
				&shmem->ints[0]))
		return NULL;
	if (std::isnan(shmem->move.v0))
		shmem->move.v0 = INFINITY;
	if (shmem->move.v0 == 0) {
		debug("Invalid speed");
		exit(1);
		Py_RETURN_TRUE;
	}
	send_to_child(CMD_MOVE);
	if (shmem->ints[1])
		Py_RETURN_FALSE;
	else
		Py_RETURN_TRUE;
}

void parse_error(void *errors, char const *format, ...) {
	va_list ap;
	va_start(ap, format);
	int size = vsnprintf(NULL, 0, format, ap);
	va_end(ap);
	va_start(ap, format);
	vfprintf(stderr, format, ap);
	fprintf(stderr, "\n");
	va_end(ap);
	va_start(ap, format);
	char buffer[size + 1];
	vsnprintf(buffer, size + 1, format, ap);
	va_end(ap);
	PyObject *msg = Py_BuildValue("s#", buffer, size);
	PyList_Append(reinterpret_cast <PyObject *>(errors), msg);
}

static PyObject *parse_gcode(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// Convert a file of G-Code into a machine readable file.
	char *infile, *outfile;
	if (!PyArg_ParseTuple(args, "yy", &infile, &outfile))
		return NULL;
	// This is not sent to the child, so that the child can remain running a job while the G-Code is being parsed.
	PyObject *errors = PyList_New(0);
	parse_gcode(infile, outfile, errors);
	return errors;
}

static PyObject *run_file(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// Run commands from a file.
	const char *file = "", *probe = "";
	shmem->ints[0] = 0;
	if (!PyArg_ParseTuple(args, "|yypdd", &file, &probe, &shmem->ints[0], &shmem->floats[0], &shmem->floats[1]))
		return NULL;
	strncpy(const_cast <char *> (shmem->strs[0]), file, PATH_MAX);
	strncpy(const_cast <char *> (shmem->strs[1]), probe, PATH_MAX);
	//debug("running file %s", file);
	send_to_child(CMD_RUN);
	Py_RETURN_NONE;
}

static PyObject *sleep(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// Enable or disable motor current
	if (!PyArg_ParseTuple(args, "pp", &shmem->ints[0], &shmem->ints[1]))
		return NULL;
	send_to_child(CMD_SLEEP);
	Py_RETURN_NONE;
}

static PyObject *settemp(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// set target temperature and enable control
	if (!PyArg_ParseTuple(args, "id", &shmem->ints[0], &shmem->floats[0]))
		return NULL;
	send_to_child(CMD_SETTEMP);
	Py_RETURN_NONE;
}

static PyObject *waittemp(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// wait for a temperature sensor to reach a target range
	if (!PyArg_ParseTuple(args, "idd", &shmem->ints[0], &shmem->floats[0], &shmem->floats[1]))
		return NULL;
	send_to_child(CMD_WAITTEMP);
	Py_RETURN_NONE;
}

static PyObject *temp_value(PyObject *Py_UNUSED(self), PyObject *args) {
	//FUNCTION_START;
	// read temperature
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_TEMP_VALUE);
	return PyFloat_FromDouble(shmem->floats[0]);
}

static PyObject *power_value(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// read used power
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_POWER_VALUE);
	return Py_BuildValue("ii", shmem->ints[1], shmem->ints[2]);
}

static PyObject *setpos(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	// Set current position
	if (!PyArg_ParseTuple(args, "iid", &shmem->ints[0], &shmem->ints[1], &shmem->floats[0]))
		return NULL;
	send_to_child(CMD_SETPOS);
	Py_RETURN_NONE;
}

static PyObject *getpos(PyObject *Py_UNUSED(self), PyObject *args) {
	//FUNCTION_START;
	// Get current position
	if (!PyArg_ParseTuple(args, "ii", &shmem->ints[0], &shmem->ints[1]))
		return NULL;
	send_to_child(CMD_GETPOS);
	return PyFloat_FromDouble(shmem->floats[0]);
}

static PyObject *read_globals(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_READ_GLOBALS);
	// These values are needed in this process for parsing G-Code
	max_deviation = shmem->floats[1];
	max_v = shmem->floats[2];
	max_a = shmem->floats[3];
	max_J = shmem->floats[4];
	return Py_BuildValue("{si,si,si,si,si,si,si,si,si,si,si,si,si,si,si,si,sd,sd,sd,sd,sd,sd,sd,sd}",
			"queue_length", shmem->ints[0],
			"num_pins", shmem->ints[1],
			"num_temps", shmem->ints[2],
			"num_gpios", shmem->ints[3],
			"led_pin", shmem->ints[4],
			"stop_pin", shmem->ints[5],
			"probe_pin", shmem->ints[6],
			"spiss_pin", shmem->ints[7],
			"pattern_step_pin", shmem->ints[8],
			"pattern_dir_pin", shmem->ints[9],
			"timeout", shmem->ints[10],
			"bed_id", shmem->ints[11],
			"fan_id", shmem->ints[12],
			"spindle_id", shmem->ints[13],
			"current_extruder", shmem->ints[14],
			"store_adc", shmem->ints[15],
			"feedrate", shmem->floats[0],
			"max_deviation", shmem->floats[1],
			"max_v", shmem->floats[2],
			"max_a", shmem->floats[3],
			"max_J", shmem->floats[4],
			"targetx", shmem->floats[5],
			"targety", shmem->floats[6],
			"targetangle", shmem->floats[7],
			"zoffset", shmem->floats[8]);
}

static void set_int(int num, char const *name, PyObject *dict) {
	PyObject *value = PyMapping_GetItemString(dict, name);
	if (!value) {
		debug("not setting value for %s, which is not in the dict", name);
		return;
	}
	PyMapping_DelItemString(dict, name);
	shmem->ints[num] = PyLong_AsLong(value);
	Py_DECREF(value);
}

static void set_float(int num, char const *name, PyObject *dict) {
	PyObject *value = PyMapping_GetItemString(dict, name);
	if (!value)
		return;
	PyMapping_DelItemString(dict, name);
	shmem->floats[num] = PyFloat_AsDouble(value);
	Py_DECREF(value);
}

static PyObject *get_object(char const *name, PyObject *dict) {
	FUNCTION_START;
	PyObject *value = PyMapping_GetItemString(dict, name);
	if (!value) {
		debug("Requested object %s not found", name);
		return NULL;
	}
	PyMapping_DelItemString(dict, name);
	return value;
}

static PyObject *write_globals(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	if (!PyArg_ParseTuple(args, "O!", &PyDict_Type, &dict))
		return NULL;
	send_to_child(CMD_READ_GLOBALS);
	set_int(2, "num_temps", dict);
	set_int(3, "num_gpios", dict);
	set_int(4, "led_pin", dict);
	set_int(5, "stop_pin", dict);
	set_int(6, "probe_pin", dict);
	set_int(7, "spiss_pin", dict);
	set_int(8, "pattern_step_pin", dict);
	set_int(9, "pattern_dir_pin", dict);
	set_int(10, "timeout", dict);
	set_int(11, "bed_id", dict);
	set_int(12, "fan_id", dict);
	set_int(13, "spindle_id", dict);
	set_int(14, "current_extruder", dict);
	set_int(15, "store_adc", dict);
	set_float(0, "feedrate", dict);
	set_float(1, "max_deviation", dict);
	set_float(2, "max_v", dict);
	set_float(3, "max_a", dict);
	set_float(4, "max_J", dict);
	set_float(5, "targetx", dict);
	set_float(6, "targety", dict);
	set_float(7, "targetangle", dict);
	set_float(8, "zoffset", dict);
	send_to_child(CMD_WRITE_GLOBALS);
	return assert_empty_dict(dict, "write_globals");
}

static PyObject *read_module_data() {
	int num_int = shmem->ints[100];
	int num_float = shmem->ints[101];
	PyObject *ints = PyTuple_New(num_int);
	for (int i = 0; i < num_int; ++i)
		PyTuple_SET_ITEM(ints, i, PyLong_FromLong(shmem->ints[102 + i]));
	PyObject *floats = PyTuple_New(num_float);
	for (int i = 0; i < num_float; ++i)
		PyTuple_SET_ITEM(floats, i, PyFloat_FromDouble(shmem->floats[100 + i]));
	PyObject *ret = Py_BuildValue("OO", ints, floats);
	Py_DECREF(ints);
	Py_DECREF(floats);
	return ret;
}

static PyObject *read_space_info(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_READ_SPACE_INFO);
	PyObject *module_data = read_module_data();
	PyObject *ret = Py_BuildValue("{si,si,si,sO}",
			"type", shmem->ints[1],
			"num_axes", shmem->ints[2],
			"num_motors", shmem->ints[3],
			"module", module_data);
	return ret;
}

static PyObject *read_space_axis(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	int type;
	if (!PyArg_ParseTuple(args, "iii", &shmem->ints[0], &shmem->ints[1], &type))
		return NULL;
	send_to_child(CMD_READ_SPACE_AXIS);
	PyObject *module_data = read_module_data();
	return Py_BuildValue("{si,sd,sd,sd,sO}",
			"park_order", shmem->ints[2],
			"park", shmem->floats[0],
			"min", shmem->floats[1],
			"max", shmem->floats[2],
			"module", module_data);
}

static PyObject *read_space_motor(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	int type;
	if (!PyArg_ParseTuple(args, "iii", &shmem->ints[0], &shmem->ints[1], &type))
		return NULL;
	send_to_child(CMD_READ_SPACE_MOTOR);
	PyObject *module_data = read_module_data();
	return Py_BuildValue("{si,si,si,si,si,si,sd,sd,sd,sd,sO}",
			"step_pin", shmem->ints[2],
			"dir_pin", shmem->ints[3],
			"enable_pin", shmem->ints[4],
			"limit_min_pin", shmem->ints[5],
			"limit_max_pin", shmem->ints[6],
			"home_order", shmem->ints[7],
			"steps_per_unit", shmem->floats[0],
			"home_pos", shmem->floats[1],
			"limit_v", shmem->floats[2],
			"limit_a", shmem->floats[3],
			"module", module_data);
}

#define assert_type(type, value) do { if (! Py ## type ## _Check (value)) { debug("python type check failed: " #type); abort();}} while (0)

static void write_module_data(PyObject *module) {
	// Write module data from module object into shmem at offset.
	// Steals reference to module.
	int int_index = 102;
	int float_index = 100;
	int num = PySequence_Size(module);
	if (num < 0) {
		abort();
	}
	for (int i = 0; i < num; ++i) {
		PyObject *value = PySequence_GetItem(module, i);
		if (PyLong_Check(value)) {
			shmem->ints[int_index++] = PyLong_AsLong(value);
		}
		else {
			assert_type(Float, value);
			shmem->floats[float_index++] = PyFloat_AsDouble(value);
		}
		Py_DECREF(value);
	}
	shmem->ints[100] = int_index - 102;
	shmem->ints[101] = float_index - 100;
	Py_DECREF(module);
}

static PyObject *write_space_info(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	if (!PyArg_ParseTuple(args, "iO!", &shmem->ints[0], &PyDict_Type, &dict))
		return NULL;
	send_to_child(CMD_READ_SPACE_INFO);
	set_int(1, "type", dict);
	set_int(2, "num_axes", dict);
	PyObject *module = get_object("module", dict);
	write_module_data(module);
	send_to_child(CMD_WRITE_SPACE_INFO);
	return assert_empty_dict(dict, "write_space_info");
}

static PyObject *write_space_axis(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	int type;
	if (!PyArg_ParseTuple(args, "iiO!i", &shmem->ints[0], &shmem->ints[1], &PyDict_Type, &dict, &type))
		return NULL;
	send_to_child(CMD_READ_SPACE_AXIS);
	set_int(2, "park_order", dict);
	set_float(0, "park", dict);
	set_float(1, "min", dict);
	set_float(2, "max", dict);
	PyObject *module = get_object("module", dict);
	write_module_data(module);
	send_to_child(CMD_WRITE_SPACE_AXIS);
	return assert_empty_dict(dict, "write_space_axis");
}

static PyObject *write_space_motor(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	int type;
	if (!PyArg_ParseTuple(args, "iiO!i", &shmem->ints[0], &shmem->ints[1], &PyDict_Type, &dict, &type))
		return NULL;
	send_to_child(CMD_READ_SPACE_MOTOR);
	set_int(2, "step_pin", dict);
	set_int(3, "dir_pin", dict);
	set_int(4, "enable_pin", dict);
	set_int(5, "limit_min_pin", dict);
	set_int(6, "limit_max_pin", dict);
	set_int(7, "home_order", dict);
	set_float(0, "steps_per_unit", dict);
	set_float(1, "home_pos", dict);
	set_float(2, "limit_v", dict);
	set_float(3, "limit_a", dict);
	PyObject *module = get_object("module", dict);
	write_module_data(module);
	send_to_child(CMD_WRITE_SPACE_MOTOR);
	return assert_empty_dict(dict, "write_space_motor");
}

static PyObject *read_temp(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_READ_TEMP);
	return Py_BuildValue("{si,si,si,sd,sd,sd,sd,sd,sd,sd,sd,sd,sd,sd,sd}",
			"heater_pin", shmem->ints[1],
			"fan_pin", shmem->ints[2],
			"thermistor_pin", shmem->ints[3],
			"R0", shmem->floats[0],
			"R1", shmem->floats[1],
			"logRc", shmem->floats[2],
			"Tc", shmem->floats[3],
			"beta", shmem->floats[4],
			"fan_temp", shmem->floats[5],
			"fan_duty", shmem->floats[6],
			"heater_limit_l", shmem->floats[7],
			"heater_limit_h", shmem->floats[8],
			"fan_limit_l", shmem->floats[9],
			"fan_limit_h", shmem->floats[10],
			"hold_time", shmem->floats[11]);
}

static PyObject *write_temp(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	if (!PyArg_ParseTuple(args, "iO!", &shmem->ints[0], &PyDict_Type, &dict))
		return NULL;
	send_to_child(CMD_READ_TEMP);
	set_int(1, "heater_pin", dict);
	set_int(2, "fan_pin", dict);
	set_int(3, "thermistor_pin", dict);
	set_float(0, "R0", dict);
	set_float(1, "R1", dict);
	set_float(2, "logRc", dict);
	set_float(3, "Tc", dict);
	set_float(4, "beta", dict);
	set_float(5, "fan_temp", dict);
	set_float(6, "fan_duty", dict);
	set_float(7, "heater_limit_l", dict);
	set_float(8, "heater_limit_h", dict);
	set_float(9, "fan_limit_l", dict);
	set_float(10, "fan_limit_h", dict);
	set_float(11, "hold_time", dict);
	send_to_child(CMD_WRITE_TEMP);
	return assert_empty_dict(dict, "write_temp");
}

static PyObject *read_gpio(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_READ_GPIO);
	return Py_BuildValue("{si,si,si,si,si,sd}",
			"pin", shmem->ints[1],
			"state", shmem->ints[2],
			"space", shmem->ints[3],
			"motor", shmem->ints[4],
			"ticks", shmem->ints[5],
			"duty", shmem->floats[0]);
}

static PyObject *write_gpio(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	PyObject *dict;
	if (!PyArg_ParseTuple(args, "iO!", &shmem->ints[0], &PyDict_Type, &dict))
		return NULL;
	send_to_child(CMD_READ_GPIO);
	set_int(1, "pin", dict);
	set_int(2, "state", dict);
	set_int(3, "space", dict);
	set_int(4, "motor", dict);
	set_int(5, "ticks", dict);
	set_float(0, "duty", dict);
	send_to_child(CMD_WRITE_GPIO);
	return assert_empty_dict(dict, "write_gpio");
}

static PyObject *queued(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "p", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_QUEUED);
	return Py_BuildValue("i", shmem->ints[1]);
}

static PyObject *home(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	shmem->ints[0] = PyTuple_Size(args);
	for (int i = 0; i < shmem->ints[0]; ++i) {
		PyObject *value = PyTuple_GetItem(args, i);
		shmem->ints[i + 1] = PyLong_AsLong(value);
	}
	send_to_child(CMD_HOME);
	Py_RETURN_NONE;
}

static PyObject *pin_value(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "i", &shmem->ints[0]))
		return NULL;
	send_to_child(CMD_PIN_VALUE);
	if (shmem->ints[1])
		Py_RETURN_TRUE;
	else
		Py_RETURN_FALSE;
}

static PyObject *pause(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_PAUSE);
	Py_RETURN_NONE;
}

static PyObject *resume(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_RESUME);
	Py_RETURN_NONE;
}

static PyObject *unpause(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_UNPAUSE);
	Py_RETURN_NONE;
}

static PyObject *get_time(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_GET_TIME);
	return Py_BuildValue("d", shmem->floats[0]);
}

static PyObject *spi(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	const char *data;
	if (!PyArg_ParseTuple(args, "iy", &shmem->ints[0], &data))
		return NULL;
	strncpy(const_cast <char *>(shmem->strs[0]), data, PATH_MAX);
	send_to_child(CMD_SPI);
	Py_RETURN_NONE;
}

static PyObject *adjust_probe(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "ddd", &shmem->floats[0], &shmem->floats[1], &shmem->floats[2]))
		return NULL;
	send_to_child(CMD_ADJUST_PROBE);
	Py_RETURN_NONE;
}

static PyObject *tp_getpos(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	send_to_child(CMD_TP_GETPOS);
	return Py_BuildValue("d", shmem->floats[0]);
}

static PyObject *tp_setpos(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "d", &shmem->floats[0]))
		return NULL;
	send_to_child(CMD_TP_SETPOS);
	Py_RETURN_NONE;
}

static PyObject *tp_findpos(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, "ddd", &shmem->floats[0], &shmem->floats[1], &shmem->floats[2]))
		return NULL;
	send_to_child(CMD_TP_FINDPOS);
	return Py_BuildValue("d", shmem->floats[3]);
}

static PyObject *motors2xyz(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	shmem->ints[1] = PyTuple_Size(args) - 1;
	if (shmem->ints[1] < 0) {
		PyErr_SetString(PyExc_ValueError, "motors2xyz needs space as first argument");
		return NULL;
	}
	PyObject *value = PyTuple_GetItem(args, 0);
	shmem->ints[0] = PyLong_AsLong(value);
	for (int i = 0; i < shmem->ints[1]; ++i) {
		value = PyTuple_GetItem(args, i + 1);
		shmem->floats[i] = PyFloat_AsDouble(value);
	}
	send_to_child(CMD_MOTORS2XYZ);
	value = PyTuple_New(shmem->ints[0]);
	for (int i = 0; i < shmem->ints[0]; ++i) {
		PyObject *f = PyFloat_FromDouble(shmem->floats[shmem->ints[1] + i]);
		PyTuple_SET_ITEM(value, i, f);
	}
	return value;
}

static PyObject *fileno(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	return Py_BuildValue("i", interrupt);
}

static PyObject *get_interrupt(PyObject *Py_UNUSED(self), PyObject *args) {
	//FUNCTION_START;	// Disable this because there is a more informative message below.
	if (!PyArg_ParseTuple(args, ""))
		return NULL;
	char c;
	if (read(interrupt, &c, 1) != 1) {
		debug("unable to read interrupt");
		exit(1);
	}
	PyObject *ret;
	if (DEBUG_FUNCTIONS) {
		char const *interrupt_cmd_name[] = { "LIMIT", "FILE_DONE", "MOVECB", "HOMED", "TIMEOUT", "PINCHANGE", "PINNAME", "DISCONNECT", "UPDATE_PIN", "UPDATE_TEMP", "CONFIRM", "PARKWAIT", "CONNECTED", "TEMPCB", "MESSAGE" };
		debug("interrupt received: %s", unsigned(c) < sizeof(interrupt_cmd_name) / sizeof(*interrupt_cmd_name) ? interrupt_cmd_name[unsigned(c)] : "(invalid)");
	}
	switch (c) {
	case CMD_LIMIT:
		ret = Py_BuildValue("{ss,si,si,sd}", "type", "limit", "space", shmem->interrupt_ints[0], "motor", shmem->interrupt_ints[1], "pos", shmem->interrupt_float);
		break;
	case CMD_FILE_DONE:
		ret = Py_BuildValue("{ss}", "type", "file-done");
		break;
	case CMD_MOVECB:
		ret = Py_BuildValue("{ss}", "type", "move-cb");
		break;
	case CMD_HOMED:
		ret = Py_BuildValue("{ss}", "type", "homed");
		break;
	case CMD_TIMEOUT:
		ret = Py_BuildValue("{ss}", "type", "timeout");
		break;
	case CMD_PINCHANGE:
		ret = Py_BuildValue("{ss,si,si}", "type", "pinchange", "pin", shmem->interrupt_ints[0], "state", shmem->interrupt_ints[1]);
		break;
	case CMD_PINNAME:
		//debug("name for pin %d: %x %s %d", shmem->interrupt_ints[0], shmem->interrupt_str[0], &shmem->interrupt_str[1], shmem->interrupt_ints[1]);
		ret = Py_BuildValue("{ss,si,si,sy#}", "type", "pinname", "pin", shmem->interrupt_ints[0], "mode", shmem->interrupt_str[0], "name", &shmem->interrupt_str[1], shmem->interrupt_ints[1]);
		break;
	case CMD_DISCONNECT:
		ret = Py_BuildValue("{ss,ss}", "type", "disconnect", "reason", shmem->interrupt_str);
		break;
	case CMD_UPDATE_PIN:
		ret = Py_BuildValue("{ss,si,si}", "type", "update-pin", "pin", shmem->interrupt_ints[0], "state", shmem->interrupt_ints[1]);
		break;
	case CMD_UPDATE_TEMP:
		ret = Py_BuildValue("{ss,si,sd}", "type", "update-temp", "temp", shmem->interrupt_ints[0], "value", shmem->interrupt_float);
		break;
	case CMD_CONFIRM:
		ret = Py_BuildValue("{ss,si,ss#}", "type", "confirm", "tool-changed", shmem->interrupt_ints[0], "message", shmem->interrupt_str, shmem->interrupt_ints[1]);
		break;
	case CMD_PARKWAIT:
		ret = Py_BuildValue("{ss}", "type", "park");
		break;
	case CMD_CONNECTED:
		ret = Py_BuildValue("{ss}", "type", "connected");
		break;
	case CMD_TEMPCB:
		ret = Py_BuildValue("{ss,si}", "type", "temp-cb", "temp", shmem->interrupt_ints[0]);
		break;
	case CMD_MESSAGE:
		ret = Py_BuildValue("{ss}", "message", shmem->interrupt_str[0]);
		break;
	default:
		PyErr_Format(PyExc_AssertionError, "Interrupt returned unexpected code 0x{x}", c);
		ret = NULL;
		break;
	}
	cdebug("interrupt received, sending reply");
	if (write(interrupt_reply, &c, 1) != 1)
		exit(1);
	cdebug("reply sent");
	return ret;
}

PyObject *init_module(PyObject *Py_UNUSED(self), PyObject *args) {
	FUNCTION_START;
	char const *cdriver;
	char const *typepath;
	if (!PyArg_ParseTuple(args, "yy", &cdriver, &typepath))
		return NULL;
	int pipe_toserver[2], pipe_fromserver[2], pipe_interrupt[2], pipe_interrupt_reply[2];
	memfd = memfd_create("shmem", 0);
	if (memfd < 0) {
		debug("failed to create memfd");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	if (ftruncate(memfd, sizeof(SharedMemory))) {
		debug("failed to resize memfd");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	shmem = reinterpret_cast <SharedMemory *>(mmap(NULL, sizeof(SharedMemory), PROT_READ | PROT_WRITE, MAP_SHARED, memfd, 0));
	strncpy(const_cast <char *>(shmem->strs[0]), typepath, PATH_MAX);
	if (shmem == NULL) {
		debug("failed to map shared memory");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	if (pipe(pipe_toserver)) {
		debug("failed to create pipe");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	if (pipe(pipe_fromserver)) {
		debug("failed to create pipe");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	if (pipe(pipe_interrupt)) {
		debug("failed to create pipe");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	if (pipe(pipe_interrupt_reply)) {
		debug("failed to create pipe");
		return PyErr_SetFromErrno(PyExc_ImportError);
	}
	shmem->ints[0] = pipe_toserver[1];
	shmem->ints[1] = pipe_fromserver[0];
	shmem->ints[2] = pipe_interrupt[1];
	shmem->ints[3] = pipe_interrupt_reply[0];
	switch (fork()) {
		case -1:
			// Error.
			debug("unable to fork");
			return PyErr_SetFromErrno(PyExc_ImportError);
		case 0:
		{
			// Child.
			close(pipe_toserver[0]);
			close(pipe_fromserver[1]);
			close(pipe_interrupt[0]);
			close(pipe_interrupt_reply[1]);
			char memfd_str[10];
			snprintf(memfd_str, sizeof(memfd_str), "%d", memfd);
			char *const arguments[] = {(char *)cdriver, memfd_str, NULL};
			execv(cdriver, arguments);
		}
		default:
			// Parent.
			close(pipe_toserver[1]);
			close(pipe_fromserver[0]);
			close(pipe_interrupt[1]);
			close(pipe_interrupt_reply[0]);
			toserver = pipe_toserver[0];
			fromserver = pipe_fromserver[1];
			interrupt = pipe_interrupt[0];
			interrupt_reply = pipe_interrupt_reply[1];
			send_to_child(-1);
	}
	Py_RETURN_NONE;
}

static PyMethodDef Methods[] = {
#ifdef SERIAL
	{"set_uuid", set_uuid, METH_VARARGS, "Set machine UUID."},
	{"get_uuid", get_uuid, METH_VARARGS, "Get machine UUID."},
	{"force_disconnect", force_disconnect, METH_VARARGS, "Disconnect serial port to machine."},
	{"connect_machine", connect_machine, METH_VARARGS, "Connect a machine."},
	{"reconnect", reconnect, METH_VARARGS, "Reconnect a machine."},
#endif
	{"move", reinterpret_cast<PyCFunction>(move), METH_VARARGS | METH_KEYWORDS, "Queue a move."},
	{"parse_gcode", parse_gcode, METH_VARARGS, "Parse a file of G-Code."},
	{"run_file", run_file, METH_VARARGS, "Run a parsed file."},
	{"sleep", sleep, METH_VARARGS, "Disable the motors."},
	{"settemp", settemp, METH_VARARGS, "Set temperature target."},
	{"waittemp", waittemp, METH_VARARGS, "Wait for a temperature control to reach its target."},
	{"temp_value", temp_value, METH_VARARGS, "Read current temperature."},
	{"power_value", power_value, METH_VARARGS, "Read power usage for a temperature control."},
	{"setpos", setpos, METH_VARARGS, "Set current extruder position."},
	{"getpos", getpos, METH_VARARGS, "Get current position of an axis."},
	{"read_globals", read_globals, METH_VARARGS, "Read global variables."},
	{"write_globals", write_globals, METH_VARARGS, "Set global variables."},
	{"read_space_info", read_space_info, METH_VARARGS, "Read settings for a space."},
	{"read_space_axis", read_space_axis, METH_VARARGS, "Read settings for an axis."},
	{"read_space_motor", read_space_motor, METH_VARARGS, "Read settings for a motor."},
	{"write_space_info", write_space_info, METH_VARARGS, "Set up a space."},
	{"write_space_axis", write_space_axis, METH_VARARGS, "Set up an axis."},
	{"write_space_motor", write_space_motor, METH_VARARGS, "Set up a motor."},
	{"read_temp", read_temp, METH_VARARGS, "Read settings for a temperature control."},
	{"write_temp", write_temp, METH_VARARGS, "Set up a temperature control."},
	{"read_gpio", read_gpio, METH_VARARGS, "Read settings for a gpio."},
	{"write_gpio", write_gpio, METH_VARARGS, "Set up a gpio."},
	{"queued", queued, METH_VARARGS, "Query number of queued commands."},
	{"home", home, METH_VARARGS, "Slowly move away from limit switches."},
	{"pin_value", pin_value, METH_VARARGS, "Get state of gpio pin."},
	{"pause", pause, METH_VARARGS, "Pause the job."},
	{"resume", resume, METH_VARARGS, "Resume moving after pausing."},
	{"unpause", unpause, METH_VARARGS, "Discard resume information."},
	{"get_time", get_time, METH_VARARGS, "Get estimate for remaining time."},
	{"spi", spi, METH_VARARGS, "Send SPI command."},
	{"adjust_probe", adjust_probe, METH_VARARGS, "Change offset for probe."},
	{"tp_getpos", tp_getpos, METH_VARARGS, "Get current position in toolpath."},
	{"tp_setpos", tp_setpos, METH_VARARGS, "Set position in toolpath."},
	{"tp_findpos", tp_findpos, METH_VARARGS, "Find position in toolpath closest to a point."},
	{"motors2xyz", motors2xyz, METH_VARARGS, "Convert motor positions to tool position."},
	{"fileno", fileno, METH_VARARGS, "Get file descriptor which will signal asynchronous events."},
	{"get_interrupt", get_interrupt, METH_VARARGS, "Read and parse interrupt from chlid process."},
	{"init", init_module, METH_VARARGS, "Initialize module and start child process."},
	{NULL, NULL, 0, NULL}
};

static PyModuleDef Module = {
	PyModuleDef_HEAD_INIT,
	"cdriver",
	NULL,
	-1,
	Methods
};

extern "C" {
	PyMODINIT_FUNC PyInit_cdriver() {
		return PyModule_Create(&Module);
	}
}
