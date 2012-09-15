#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "Simulator.hh"
#include "Commands.hh"
#include "s3g_private.h"
#include "s3g_stdio.h"
#include "s3g.h"

typedef struct {
     union {
	  int32_t i;
	  uint32_t u;
	  unsigned char c[4];
     } u;
} foo_32_t;

typedef struct {
     uint8_t     cmd_id;
     size_t      cmd_len;
     const char *cmd_name;
} s3g_command_info_t;

static const s3g_command_info_t command_table_raw[] = {
     /*   0 */  {HOST_CMD_VERSION, 0, "version"},
     /*   1 */  {HOST_CMD_INIT, 0, "initialize"},
     /*   2 */  {HOST_CMD_GET_BUFFER_SIZE, 0, "get buffer size"},
     /*   3 */  {HOST_CMD_CLEAR_BUFFER, 0, "clear buffer"},
     /*   4 */  {HOST_CMD_GET_POSITION, 0, "get position"},
     /*   5 */  {HOST_CMD_GET_RANGE, 0, "get range"},
     /*   6 */  {HOST_CMD_SET_RANGE, 0, "set range"},
     /*   7 */  {HOST_CMD_ABORT, 0, "abort"},
     /*   8 */  {HOST_CMD_PAUSE, 0, "pause"},
     /*   9 */  {HOST_CMD_PROBE, 0, "probe"},
     /*  10 */  {HOST_CMD_TOOL_QUERY, 0, "tool query"},
     /*  11 */  {HOST_CMD_IS_FINISHED, 0, "is finished?"},
     /*  12 */  {HOST_CMD_READ_EEPROM, 0, "read EEPROM"},
     /*  13 */  {HOST_CMD_WRITE_EEPROM, 0, "write EEPROM"},
     /*  14 */  {HOST_CMD_CAPTURE_TO_FILE, 0, "capture to file"},
     /*  15 */  {HOST_CMD_END_CAPTURE, 0, "end capture"},
     /*  16 */  {HOST_CMD_PLAYBACK_CAPTURE, 0, "playback capture"},
     /*  17 */  {HOST_CMD_RESET, 0, "sortware reset"},
     /*  18 */  {HOST_CMD_NEXT_FILENAME, 0, "next SD card filename"},
     /*  19 */  {HOST_CMD_GET_DBG_REG, 0, "get debug register"},
     /*  20 */  {HOST_CMD_GET_BUILD_NAME, 0, "get build name"},
     /*  21 */  {HOST_CMD_GET_POSITION_EXT, 0, "get position extended"},
     /*  22 */  {HOST_CMD_EXTENDED_STOP, 0, "extended stop"},
     /*  25 */  {HOST_CMD_GET_COMMUNICATION_STATS, 0, "get communication stats"},
     /* 112 */  {HOST_CMD_DEBUG_ECHO, 0, "debug echo"},
     /* 129 */  {HOST_CMD_QUEUE_POINT_ABS, 16, "queue point absolute"},
     /* 130 */  {HOST_CMD_SET_POSITION, 12, "set position"},
     /* 131 */  {HOST_CMD_FIND_AXES_MINIMUM, 7, "find axes minimum"},
     /* 132 */  {HOST_CMD_FIND_AXES_MAXIMUM, 7, "find axes maximum"},
     /* 133 */  {HOST_CMD_DELAY, 4, "delay"},
     /* 134 */  {HOST_CMD_CHANGE_TOOL, 1, "change tool"},
     /* 135 */  {HOST_CMD_WAIT_FOR_TOOL, 5, "wait for tool ready"},
     /* 136 */  {HOST_CMD_TOOL_COMMAND, 0xffffffff, "tool action"},
     /* 137 */  {HOST_CMD_ENABLE_AXES, 1, "enable/disable axes"},
     /* 138 */  {138, 2, "user block"},
     /* 139 */  {HOST_CMD_QUEUE_POINT_EXT, 24, "queue point extended"},
     /* 140 */  {HOST_CMD_SET_POSITION_EXT, 20, "set position extended"},
     /* 141 */  {HOST_CMD_WAIT_FOR_PLATFORM, 5, "wait for platform ready"},
     /* 142 */  {HOST_CMD_QUEUE_POINT_NEW, 25, "queue new point"},
     /* 143 */  {HOST_CMD_STORE_HOME_POSITION, 1, "store home position"},
     /* 144 */  {HOST_CMD_RECALL_HOME_POSITION, 1, "recall home position"},
     /* 201 */  {HOST_CMD_SET_MAX_ACCEL	, 0, "set maximum accelerations"},
     /* 203 */  {HOST_CMD_SET_MAX_FEEDRATE, 0, "set maximum feed rates"},
     /* 204 */  {HOST_CMD_SET_DEFAULT_ACCEL, 0, "set default accelerations"},
     /* 205 */  {HOST_CMD_SET_ADVANCED_ACCEL, 0, "set advanced accelerations"},
     /* 206 */  {HOST_CMD_SET_ADVANCED_ACCEL2, 0, "set filament diameter"},
     /* 207 */  {HOST_CMD_SET_ADVANCE_K	, 0, "set 'advance' K constant"},
     /* 208 */  {HOST_CMD_SET_EXTRUDER_STEPSMM, 0, "set extruder steps per mm"},
     /* 209 */  {HOST_CMD_SET_ACCELERATION, 0, "enable/disable acceleration"},
     /* 210 */  {HOST_CMD_MOOD_LIGHT_SET_RGB, 0, "set mood light RGB"},
     /* 211 */  {HOST_CMD_MOOD_LIGHT_SET_HSB, 0, "set mood light HSB"},
     /* 212 */  {HOST_CMD_MOOD_LIGHT_PLAY_SCRIPT, 0, "play mood light script"},
     /* 213 */  {HOST_CMD_BUZZER_REPEATS, 0, "set buzzer repeats"},
     /* 214 */  {HOST_CMD_BUZZER_BUZZ, 0, "enable/disable buzzer"},
     /* 215 */  {HOST_CMD_SET_AXIS_STEPS_MM, 0, "set axis steps per mm"}
};

static s3g_command_info_t command_table[256];

// Not thread safe

static int table_initialized = 0;

static int s3g_init(void)
{
     int i, istat;
     const s3g_command_info_t *p;

     if (table_initialized != 0)
	  return(0);

     // Initialize the indexed command table
     memset(command_table, 0, sizeof(command_table));

     istat = 0;
     p = command_table_raw;

     // Load the indexed command table, looking for conflicts
     for (i = 0; i < sizeof(command_table_raw) / sizeof(s3g_command_info_t); i++, p++)
     {
	  if ((command_table[p->cmd_id].cmd_len != 0 ||
	       command_table[p->cmd_id].cmd_name != NULL))
	  {
	       // Table already has an entry for this command id
	       // Make sure that the lengths don't conflict
	       if (command_table[p->cmd_id].cmd_len != p->cmd_len)
	       {
		    fprintf(stderr,
			    "s3g_init(%d): Two commands with identical ids (%d) but "
			    "different lengths encountered; ignoring \"%s\"\n",
			    __LINE__, p->cmd_id, p->cmd_name ? p->cmd_name : "<no name>");
		    istat = -1;
		    continue;
	       }
	  }
	  command_table[p->cmd_id].cmd_id   = p->cmd_id;
	  command_table[p->cmd_id].cmd_len  = p->cmd_len;
	  command_table[p->cmd_id].cmd_name = p->cmd_name;
     }

     table_initialized = -1;

     return(istat);
}


s3g_context_t *s3g_open(int type, void *src)
{
     s3g_context_t *ctx;

     (void)type; // Only type is a file

     ctx = (s3g_context_t *)calloc(1, sizeof(s3g_context_t));
     if (!ctx)
     {
	  fprintf(stderr, "s3g_open(%d): Unable to allocate VM; %s (%d)\n",
		  __LINE__, strerror(errno), errno);
	  return(NULL);
     }

     if (s3g_stdio_open(ctx, src))
	  return(NULL);

     return(ctx);
}


int s3g_close(s3g_context_t *ctx)
{
     int iret;

     if (!ctx)
	  return(0);

     iret = (ctx->close != NULL) ? (*ctx->close)(ctx->rw_ctx) : 0;

     free(ctx);

     return(iret);
}


int s3g_command_read_ext(s3g_context_t *ctx, s3g_command_t *cmd,
			 unsigned char *buf, size_t maxbuf, size_t *buflen)
{
     unsigned char *buf0 = buf;
     ssize_t bytes_expected, bytes_read;
     s3g_command_info_t *ct;
     s3g_command_t dummy;
     foo_32_t f32;
     int iret;
     uint8_t ui8arg;

     iret = -1;

     if (buflen)
	  *buflen = 0;

     // We have to have a read context
     // We don't need a command context to return the command in
     if (!ctx || !buf || maxbuf == 0)
     {
	  fprintf(stderr, "s3g_command_get(%d): Invalid call; ctx=%p, buf=%p, "
		  "maxbuf=%u\n", __LINE__, (void *)ctx, (void *)buf, maxbuf);
	  errno = EINVAL;
	  return(-1);
     }
     else if (!ctx->read)
     {
	  fprintf(stderr, "s3g_command_get(%d): Invalid context; "
		  "ctx->read=NULL\n", __LINE__);
	  errno = EINVAL;
	  return(-1);
     }
     else if (!buf || maxbuf == 0)
     {
	  fprintf(stderr, "s3g_command_get(%d): Invalid context; "
		  "ctx->read=NULL\n", __LINE__);
	  errno = EINVAL;
	  return(-1);
     }

     // Initialize command table
     s3g_init();

     if (1 != (bytes_expected = (*ctx->read)(ctx->rw_ctx, buf, maxbuf, 1)))
     {
	  // End of file condition?
	  if (bytes_expected == 0)
	       return(1); // EOF

	  fprintf(stderr,
		  "s3g_command_get(%d): Error while reading from the s3g file; "
		  "%s (%d)\n",
		  __LINE__, strerror(errno), errno);
	  return(-1);
     }

     ct = command_table + buf[0];  // &command_table[buf[0]]

     buf    += 1;
     maxbuf -= 1;

     if (!cmd)
	  cmd = &dummy;

     cmd->cmd_id   = ct->cmd_id;
     cmd->cmd_name = ct->cmd_name;
     cmd->cmd_len  = ct->cmd_len;

     if (ct->cmd_name == NULL)
     {
	  fprintf(stderr,
		  "s3g_command_get(%d): Unrecognized command, %d\n",
		  __LINE__, buf[0]);
	  goto done;
     }

     switch(cmd->cmd_id)
     {
     default :
     case HOST_CMD_CHANGE_TOOL :
     case HOST_CMD_ENABLE_AXES :
     case HOST_CMD_SET_POSITION :
     case HOST_CMD_DELAY :
     case HOST_CMD_FIND_AXES_MINIMUM :
     case HOST_CMD_FIND_AXES_MAXIMUM :
     case HOST_CMD_WAIT_FOR_TOOL :
     case HOST_CMD_WAIT_FOR_PLATFORM :
     case HOST_CMD_STORE_HOME_POSITION :
     case HOST_CMD_RECALL_HOME_POSITION :
     case HOST_CMD_SET_MAX_ACCEL :
     case HOST_CMD_SET_MAX_FEEDRATE :
     case HOST_CMD_SET_DEFAULT_ACCEL :
     case HOST_CMD_SET_ADVANCED_ACCEL :
     case HOST_CMD_SET_ADVANCED_ACCEL2 :
     case HOST_CMD_SET_ADVANCE_K :
     case HOST_CMD_SET_EXTRUDER_STEPSMM :
     case HOST_CMD_SET_ACCELERATION :
     case HOST_CMD_MOOD_LIGHT_SET_RGB :
     case HOST_CMD_MOOD_LIGHT_SET_HSB :
     case HOST_CMD_MOOD_LIGHT_PLAY_SCRIPT :
     case HOST_CMD_BUZZER_REPEATS :
     case HOST_CMD_BUZZER_BUZZ :
     case HOST_CMD_SET_AXIS_STEPS_MM :
	  // Just read the data
	  bytes_expected = (ssize_t)(ct->cmd_len & 0x7fffffff);
	  if ((bytes_read = (*ctx->read)(ctx->rw_ctx, buf, maxbuf,
					 ct->cmd_len)) != bytes_expected)
	       goto io_error;

	  buf    += bytes_read;
	  maxbuf -= bytes_read;
	  break;

#define GET_INT32(v) \
	  if (maxbuf < 4) goto trunc; \
	  if (4 != (bytes_read = (*ctx->read)(ctx->rw_ctx, buf, maxbuf, 4))) \
	       goto io_error; \
	  memcpy(&f32.u.c, buf, 4); \
	  buf    += bytes_read; \
	  maxbuf -= bytes_read; \
	  cmd->t.v = f32.u.i

#define GET_UINT8(v) \
	  if (maxbuf < 1) goto trunc; \
	  if (1 != (bytes_read = (*ctx->read)(ctx->rw_ctx, buf, maxbuf, 1))) \
	       goto io_error; \
	  ui8arg = buf[0]; \
	  buf    += bytes_read; \
	  maxbuf -= bytes_read; \
	  cmd->t.v = ui8arg

#define ZERO(v,c) cmd->t.v = (c)0

     case HOST_CMD_TOOL_COMMAND :
	  // This command is VERY MBI specific
	  if ((ssize_t)3 != (*ctx->read)(ctx->rw_ctx, buf, maxbuf, 3))
	       goto io_error;
	  if (cmd)
	       cmd->cmd_len = (size_t)buf[2];
	  cmd->t.tool.subcmd_id  = buf[1];
	  cmd->t.tool.index      = buf[0];
	  cmd->t.tool.subcmd_len = bytes_expected = (ssize_t)buf[2];
	  if ((bytes_read = (*ctx->read)(ctx->rw_ctx, buf + 3, maxbuf - 3,
					 (size_t)buf[2])) != bytes_expected)
	       goto io_error;

	  if (cmd->t.tool.subcmd_len == 1)
	       cmd->t.tool.subcmd_value = (uint16_t)buf[3];
	  else if (cmd->t.tool.subcmd_len > 1)
	       memcpy((void *)&cmd->t.tool.subcmd_value, buf + 3, sizeof(uint16_t));
	  else
	       cmd->t.tool.subcmd_value = 0;

	  maxbuf -= 3 + bytes_read;
	  buf    += 3 + bytes_read;

	  switch (cmd->t.tool.subcmd_id)
	  {
	  case SLAVE_CMD_SET_TEMP :
	       cmd->t.tool.subcmd_name = "set temperature";
	       break;

	  case SLAVE_CMD_SET_PLATFORM_TEMP :
	       cmd->t.tool.subcmd_name = "set platform temperature";
	       break;

	  case SLAVE_CMD_SET_MOTOR_1_PWM :
	       cmd->t.tool.subcmd_name = "set motor 1 PWM";
	       break;

	  case SLAVE_CMD_TOGGLE_MOTOR_1 :
	       cmd->t.tool.subcmd_name = "toggle motor 1";
	       break;

	  case SLAVE_CMD_TOGGLE_ABP :
	       cmd->t.tool.subcmd_name = "toggle ABP";
	       break;

	  case SLAVE_CMD_TOGGLE_VALVE :
	       if (cmd->t.tool.subcmd_value == 0)
		    cmd->t.tool.subcmd_name = "segment acceleration off";
	       else
		    cmd->t.tool.subcmd_name = "segment acceleration on";
	       break;

	  default :
	       cmd->t.tool.subcmd_name = "unknown subcommand";
	       break;
	  }
	  break;

     case HOST_CMD_SET_POSITION_EXT :
	  // x4, y4, z4, a4, b4 = 20 bytes
	  GET_INT32(set_position_ext.x);
	  GET_INT32(set_position_ext.y);
	  GET_INT32(set_position_ext.z);
	  GET_INT32(set_position_ext.a);
	  GET_INT32(set_position_ext.b);
	  break;

     case HOST_CMD_QUEUE_POINT_EXT :
	  // x4, y4, z4, a4, b4, dda4 = 24 bytes
	  GET_INT32(queue_point_ext.x);
	  GET_INT32(queue_point_ext.y);
	  GET_INT32(queue_point_ext.z);
	  GET_INT32(queue_point_ext.a);
	  GET_INT32(queue_point_ext.b);
	  GET_INT32(queue_point_ext.dda);
	  ZERO(queue_point_ext.dummy_rel, uint8_t);
	  break;

     case HOST_CMD_QUEUE_POINT_NEW :
	  // x4, y4, z4, a4, b4, us4, relative = 25 bytes
	  GET_INT32(queue_point_new.x);
	  GET_INT32(queue_point_new.y);
	  GET_INT32(queue_point_new.z);
	  GET_INT32(queue_point_new.a);
	  GET_INT32(queue_point_new.b);
	  GET_INT32(queue_point_new.us);
	  GET_UINT8(queue_point_new.rel);
	  break;

     case HOST_CMD_QUEUE_POINT_ABS :
	  // x4, y4, z4, dda4 = 16 bytes
	  GET_INT32(queue_point_abs.x);
	  GET_INT32(queue_point_abs.y);
	  GET_INT32(queue_point_abs.z);
	  GET_INT32(queue_point_abs.dda);
	  ZERO(queue_point_abs.dummy_a, int32_t);
	  ZERO(queue_point_abs.dummy_b, int32_t);
	  ZERO(queue_point_abs.dummy_rel, uint8_t);
	  break;
     }

#undef ZERO
#undef GET_UINT8
#undef GET_INT32

     iret = 0;
     goto done;

io_error:
     fprintf(stderr,
	     "s3g_command_get(%d): Error while reading from the s3g file; "
	     "%s (%d)\n",
	     __LINE__, strerror(errno), errno);
     iret = -1;
     goto done;

trunc:
     fprintf(stderr,
	     "s3g_command_get(%d): Caller supplied read buffer is too small",
	     __LINE__);
     iret = -1;

done:
     if (buflen)
	  *buflen = (size_t)(buf - buf0);

     return(iret);
}

int s3g_command_read(s3g_context_t *ctx, s3g_command_t *cmd)
{
     unsigned char buf[1024];
     return(s3g_command_read_ext(ctx, cmd, buf, sizeof(buf), NULL));
}
