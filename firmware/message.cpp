#include "firmware.h"

void message (uint32_t code, char *msg, uint8_t len)
{
	if (have_msg)
		debug ("Warning: losing message because new one is prepared before old one is sent");
	if (len > MAXMSGLEN)
		len = MAXMSGLEN - 1;
	msg_buffer[0] = 2 + 4 + len;
	msg_buffer[1] = CMD_MESSAGE;
	ReadFloat f;
	f.l = code;
	for (uint8_t i = 0; i < 4; ++i)
		msg_buffer[2 + i] = f.b[i];
	for (uint8_t i = 0; i < len; ++i)
		msg_buffer[2 + 4 + i] = msg[i];
	have_msg = true;
	try_send_next ();
}
