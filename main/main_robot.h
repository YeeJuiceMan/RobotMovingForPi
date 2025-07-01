#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include "../include/keypress.h"
#include "../include/import_registers.h"
#include "../include/cm.h"
#include "../include/gpio.h"
#include "../include/uart.h"
#include "../include/spi.h"
#include "../include/bsc.h"
#include "../include/pwm.h"
#include "../include/enable_pwm_clock.h"
#include "../include/io_peripherals.h"
#include "../include/wait_period.h"
#include "../include/FIFO.h"
#include "../include/MPU6050.h"
#include "../include/MPU9250.h"
#include "../include/wait_key.h"
#include <linux/videodev2.h>
#include <time.h>
#include <cairo/cairo.h>
#include <gtk/gtk.h>
#include "../include/pixel_format_RGB.h"
#include "../include/video_interface.h"
#include "../include/scale_image_data.h"
#include "../include/draw_bitmap_multiwindow.h"

#define FIFO_LENGTH 64
#define THREE_QUARTERS (FIFO_LENGTH*3/4)
#define PWM_RANGE 100
#define CHECK_AMOUNT 25
#define GET_FRAMES 10  /* the number of frame times to average when determining the FPS */
#define SCALE_REDUCTION_PER_AXIS 2   /* the image size reduction ratio (note that 640*480*3*8*FPS = your bandwidth usage, at 24FPS, that is 177MPBS) */
#define SMALL_SCALE 10
#define IMAGE_SIZE sizeof(struct image_t)/(SCALE_REDUCTION_PER_AXIS*SCALE_REDUCTION_PER_AXIS)
#define SMOL_IMAGE_SIZE IMAGE_SIZE/(SMALL_SCALE*SMALL_SCALE)


struct thread_command
{
    char type;
    char command;
    uint16_t argument;
};

FIFO_TYPE(struct thread_command, FIFO_LENGTH, fifo_t);

struct key_thread_param
{
  const char                    * name;
  struct fifo_t                 * key_fifo;
  bool                          * quit_flag;
};

struct control_thread_param
{
  const char                    * name;
  struct fifo_t                 * key_fifo;
  struct fifo_t                 * motor_lr_fifo;
  struct fifo_t                 * cam_main_fifo;
  bool                          * quit_flag;
};

struct ir_thread_param
{
  const char                    * name;
  struct io_peripherals         * io; // io peripheral
  struct fifo_t                 * left_fifo;
  struct fifo_t                 * right_fifo;
  int                           * mode;
  bool                          * ir_turn_busy; // lets threads know if turning (stops changing direction)
  bool                          * quit_flag;
};

struct motor_lr_ctrl_param
{
  const char                    * name;
  struct fifo_t                 * motor_lr_fifo;
  struct fifo_t                 * left_fifo;
  struct fifo_t                 * right_fifo;
  int                           * mode;
  bool                          * quit_flag;
};

struct motor_thread_param
{
  const char                    * name;
  struct io_peripherals         * io; // io peripheral
  struct fifo_t                 * fifo;
  bool                          * turn_busy; // lets threads know if turning (stops changing direction)
  bool                          * quit_flag;
};

struct cam_ctrl_param
{
  const char                    * name;
  struct fifo_t                 * cam_main_fifo;
  struct fifo_t                 * cam_color_fifo;
  struct fifo_t                 * cam_gray_fifo;
  struct fifo_t                 * cam_bw_fifo;
  struct fifo_t                 * cam_bw_smol_fifo;
  unsigned char                 * color_raw;
  unsigned char                 * gray_raw;
  unsigned char                 * bw_raw;
  unsigned char                 * bws_raw;
  unsigned int                  * scaled_width;
  unsigned int                  * scaled_height;
  bool                          * quit_flag;
};

struct cam_thread_param
{
  const char                    * name;
  struct fifo_t                 * fifo;
  unsigned char                 * image_raw;
  unsigned int                  * scaled_width;
  unsigned int                  * scaled_height;
  bool                          * quit_flag;
};
