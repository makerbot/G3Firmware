// s3g.h
// Structure and routine definitions for the s3g library

#ifndef S3G_H_

#define S3G_H_

#include <inttypes.h>
#include "Commands.hh"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef S3G_CONTEXT_T_
#define S3G_CONTEXT_T_
typedef void s3g_context_t;
#endif

// s3g_queue_point_abs, s3g_queue_point_ext, and s3g_queue_point_new
// all have the same size, packing and layout.  They can be cast to
// one another and the relevant fields extracted.  The dummy_ fields
// will be set to zero when appropriate.
typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t dummy_a;
     int32_t dummy_b;
     int32_t dda;
     uint8_t dummy_rel;
} s3g_queue_point_abs;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t a;
     int32_t b;
     int32_t dda;
     uint8_t dummy_rel;
} s3g_queue_point_ext;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t a;
     int32_t b;
     int32_t us;
     uint8_t rel;
} s3g_queue_point_new;

typedef struct {
     uint8_t index;
} s3g_change_tool;

typedef struct {
     uint8_t axes;
} s3g_enable_axes;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
} s3g_set_position;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t a;
     int32_t b;
} s3g_set_position_ext;

typedef struct {
     uint32_t millis;
} s3g_delay;

typedef struct {
     uint8_t  flags;
     uint32_t feed_rate; // microseconds per step
     uint16_t timeout;   // seconds
} s3g_find_axes_minimum;

typedef struct {
     uint8_t  flags;
     uint32_t feed_rate; // microseconds per step
     uint16_t timeout;   // seconds
} s3g_find_axes_maximum;

typedef struct {
     uint8_t  index;
     uint16_t ping_delay;
     uint16_t timeout;
} s3g_wait_for_tool;

typedef struct {
     uint8_t  index;
     uint16_t ping_delay;
     uint16_t timeout;
} s3g_wait_for_platform;

typedef struct {
     uint8_t axes;
} s3g_store_home_position;

typedef struct {
     uint8_t axes;
} s3g_recall_home_position;

typedef struct {
     uint8_t     index;
     uint8_t     subcmd_id;
     size_t      subcmd_len;
     const char *subcmd_name;
     uint16_t    subcmd_value;
} s3g_tool;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t a;
} s3g_set_max_accel;

typedef struct {
     int32_t x;
     int32_t y;
     int32_t z;
     int32_t a;
} s3g_set_max_feedrate;

typedef struct {
     int32_t s;
     int32_t t;
} s3g_set_default_accel;

typedef struct {
     int32_t s;
     int32_t t;
     int32_t x;
     int32_t z;
} s3g_set_advanced_accel;

typedef struct {
     int32_t d;
} s3g_set_filament_diameter;

typedef struct {
     int32_t s;
     int32_t k;
} s3g_set_advance_k;

typedef struct {
     int32_t a;
} s3g_set_extruder_stepsmm;

typedef struct {
     uint8_t s;
} s3g_set_acceleration;

typedef struct {
     int32_t r;
     int32_t g;
     int32_t b;
     int32_t fade_speed;
     int32_t write_to_eeprom;
} s3g_mood_set_rgb;

typedef struct {
     int32_t h;
     int32_t s;
     int32_t b;
     int32_t write_to_eeprom;
} s3g_mood_set_hsb;

typedef struct {
     int32_t script_id;
     int32_t write_to_eeprom;
} s3g_mood_play_script;

typedef struct {
     uint8_t repeats;
} s3g_buzzer_repeats;

typedef struct {
     uint8_t buzzes;
     uint8_t duration;
     uint8_t repeats;
} s3g_buzzer_buzz;

typedef struct {
     int64_t x; // int32_t * 10000
     int64_t y; // int32_t * 10000
     int64_t z; // int32_t * 10000
     int64_t a; // int32_t * 10000
} s3g_set_axis_steps_mm;

// s3g_command_t
// An individual command read from a .s3g file is stored in
// this data structure.  You need to know from the command id
// which member of the union to look at.

// #define constants from the command ids are available in Commands.hh

typedef struct {
     uint8_t     cmd_id;
     size_t      cmd_len;
     const char *cmd_name;
     union {
	  s3g_queue_point_abs       queue_point_abs;
	  s3g_queue_point_ext       queue_point_ext;
	  s3g_queue_point_new       queue_point_new;
	  s3g_change_tool           change_tool;
	  s3g_enable_axes           enable_axes;
	  s3g_set_position          set_position;
	  s3g_set_position_ext      set_position_ext;
	  s3g_delay                 delay;
	  s3g_find_axes_minimum     find_axes_minimum;
	  s3g_find_axes_maximum     find_axes_maximum;
	  s3g_wait_for_tool         wait_for_tool;
	  s3g_wait_for_platform     wait_for_platform;
	  s3g_store_home_position   store_home_position;
	  s3g_recall_home_position  recall_home_position;
	  s3g_tool                  tool;
	  s3g_set_max_accel         set_max_accel;
	  s3g_set_max_feedrate      set_max_feedrate;
	  s3g_set_default_accel     set_default_accel;
	  s3g_set_advanced_accel    set_advanced_accel;
	  s3g_set_filament_diameter set_filament_diameter;
	  s3g_set_advance_k         set_advance_k;
	  s3g_set_extruder_stepsmm  set_extruder_stepsmm;
	  s3g_set_acceleration      set_acceleration;
	  s3g_mood_set_rgb          mood_set_rgb;
	  s3g_mood_set_hsb          mood_set_hsb;
	  s3g_mood_play_script      mood_play_script;
	  s3g_buzzer_repeats        buzzer_repeats;
	  s3g_buzzer_buzz           buzzer_buzz;
	  s3g_set_axis_steps_mm     set_axis_steps_mm;
     } t;
} s3g_command_t;

#define S3G_INPUT_TYPE_FILE 0  // stdin or a named disk file

// Obtain an s3g_context for an input source of type S3G_INPUT_TYPE_.
// The context returned must be disposed of by calling s3g_close().
//
// Call arguments:
//
//   int type
//      Input source type.  Must be one of
//
//         S3G_INPUT_TYPE_FILE
//
//   void *src
//      Input source information for the selected input type
//
//         S3G_INPUT_TYPE_FILE -- const char *filename or NULL for stdin
//
//   Return values:
//
//     != NULL -- Success
//     == NULL -- Error; consult errno

s3g_context_t *s3g_open(int type, void *src);


// Read a single command from the s3g context, storing the result in the
// supplied s3g_command_t structure
//
// Call arguments:
//
//   s3g_context_t *ctx
//     Context obtained by calling s3g_open().
//
//   s3g_command_t *cmd
//     Pointer to a s3g_command_t structure to store the read command in.  If NULL
//     is passed for this argument, then the command is read but no information about
//     the command is returned.
//
//  Return values:
//
//    0 -- Success
//    1 -- Unrecognized command encountered; not possible to read further
//   -1 -- Read error; check errno

int s3g_command_read(s3g_context_t *ctx, s3g_command_t *cmd);

int s3g_command_read_ext(s3g_context_t *ctx, s3g_command_t *cmd,
			 unsigned char *rawbuf, size_t maxbuf, size_t *len);


// Close the s3g input source, releasing any resources
//
// Call arguments:
//
//   s3g_context_t *ctx
//     Context obtained by calling s3g_open().
//
//  Return values:
//
//    0 -- Success
//   -1 -- Close error; check errno

int s3g_close(s3g_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif
