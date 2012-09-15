#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "Simulator.hh"
#include "StepperAccelPlannerExtras.hh"
#include "StepperAccel.hh"
#include "s3g.h"

static void usage(FILE *f, const char *prog)
{
     if (f == NULL)
	  f = stderr;

     fprintf(f,
"Usage: %s [? | -h] [-s] [-d mask] [-r rate] [-u] [file]\n"
"     file -- The name of the .s3g file to dump.  If not supplied then stdin is dumped\n"
"  -d mask -- Selectively enable debugging with a bit mask \"mask\"\n"
"  -r rate -- Flag feed rates which exceed \"rate\"\n"
"       -s -- Display block initial, peak and final speeds (mm/s) along with rates\n"
"       -u -- Display significant differences between interval based and us based feed rates\n"
"    ?, -h -- This help message\n",
	     prog ? prog : "s3gdump");
}

int main(int argc, const char *argv[])
{
     char c;
     s3g_command_t cmd;
     s3g_context_t *ctx;

     plan_init_eeprom();

     simulator_use_max_feed_rate = false;
     simulator_dump_speeds = false;
     simulator_show_alt_feed_rate = false;

     while ((c = getopt(argc, (char **)argv, ":hd:r:su?")) != -1)
     {
	  switch(c)
	  {
	  // Unknown switch
	  case ':' :
	  default :
	       usage(stderr, argv[0]);
	       return(1);

	  // Explicit help request
	  case 'h' :
	  case '?' :
	       usage(stdout, argv[0]);
	       return(1);

	  // Debug
	  case 'd' :
	  {
	       char *ptr = NULL;
	       simulator_debug = (uint32_t)(0xffffffff & strtoul(optarg, &ptr, 0));
	       if (ptr == NULL || ptr == optarg)
	       {
		    fprintf(stderr, "%s: unable to parse the debug mask, \"%s\", as an integer\n",
			    argv[0], optarg);
		    return(1);
	       }
	  }
	  break;

	  // Max feed rate
	  case 'r' :
	  {
	       char *ptr = NULL;
	       float rate;

	       rate = strtof(optarg, &ptr);
	       if (ptr == NULL || ptr == optarg)
	       {
		    fprintf(stderr, "%s: unable to parse the feed rate, \"%s\", as a floating point number\n",
			    argv[0], optarg);
		    return(1);
	       }
	       simulator_use_max_feed_rate = true;
	       simulator_max_feed_rate = FTOFP(rate);
	  }
	  break;

          // Display speeds as well as rates
	  case 's' :
	       simulator_dump_speeds = true;
	       break;

          // Display significant differences between interval based and us based feed rates
	  case 'u' :
	       simulator_show_alt_feed_rate = true;
	       break;
	  }
     }

     argc -= optind;
     argv += optind;
     if (argc == 0)
	  // Open stdin
	  ctx = s3g_open(0, NULL);
     else
	  // Open the specified file
	  ctx = s3g_open(0, (void *)argv[0]);

     if (!ctx)
	  // Assume that s3g_open() has complained
	  return(1);

     while (!s3g_command_read(ctx, &cmd))
     {
	  if (cmd.cmd_id == HOST_CMD_QUEUE_POINT_NEW)
	  {
	       setTargetNew(cmd.t.queue_point_new.x, cmd.t.queue_point_new.y, cmd.t.queue_point_new.z,
			    cmd.t.queue_point_new.a, cmd.t.queue_point_new.b,
			    cmd.t.queue_point_new.us, cmd.t.queue_point_new.rel);
	       if (movesplanned() >= (BLOCK_BUFFER_SIZE >> 1)) plan_dump_current_block(1);
	  }
	  else if (cmd.cmd_id == HOST_CMD_QUEUE_POINT_EXT)
	  {
	       setTarget(cmd.t.queue_point_ext.x, cmd.t.queue_point_ext.y, cmd.t.queue_point_ext.z,
			 cmd.t.queue_point_ext.a, cmd.t.queue_point_ext.b,
			 cmd.t.queue_point_ext.dda);
	       if (movesplanned() >= (BLOCK_BUFFER_SIZE >> 1)) plan_dump_current_block(1);
	  }
	  else if (cmd.cmd_id == HOST_CMD_QUEUE_POINT_ABS)
	  {
	       setTarget(cmd.t.queue_point_abs.x, cmd.t.queue_point_abs.y, cmd.t.queue_point_abs.z, 0, 0,
			 cmd.t.queue_point_abs.dda);
	       if (movesplanned() >= (BLOCK_BUFFER_SIZE >> 1)) plan_dump_current_block(1);
	  }
	  else if (cmd.cmd_id == HOST_CMD_SET_POSITION_EXT)
	  {
	       definePosition(cmd.t.set_position_ext.x, cmd.t.set_position_ext.y, cmd.t.set_position_ext.z,
			      cmd.t.set_position_ext.a, cmd.t.set_position_ext.b);
	  }
	  else if (cmd.cmd_id == HOST_CMD_TOOL_COMMAND &&
		   cmd.t.tool.subcmd_id == SLAVE_CMD_TOGGLE_VALVE)
	  {
	       bool accel = cmd.t.tool.subcmd_value != 0;
	       printf("*** Turn segment acceleration %s ***\n", accel ? "on" : "off");
	       setSegmentAccelState(accel);
	  }
	  else
	  {
	       // Dump queued blocks?
	       if (cmd.cmd_id != HOST_CMD_TOOL_COMMAND ||
		   (cmd.t.tool.subcmd_id != SLAVE_CMD_TOGGLE_MOTOR_1 &&
		    cmd.t.tool.subcmd_id != SLAVE_CMD_TOGGLE_VALVE))
		    while (movesplanned() != 0)
			 plan_dump_current_block(1);

	       if (cmd.cmd_id != HOST_CMD_TOOL_COMMAND)
		    printf("*** %s ***\n", cmd.cmd_name);
	       else
		    printf("*** %s (%s (%d) for tool %u to %u) ***\n",
			   cmd.cmd_name, cmd.t.tool.subcmd_name, cmd.t.tool.subcmd_id,
			   cmd.t.tool.index, cmd.t.tool.subcmd_value);
	  }
     }

     // Dump any remaining blocks
     while (movesplanned() != 0)
	  plan_dump_current_block(1);

     s3g_close(ctx);

     plan_dump_run_data();

     return(0);
}
