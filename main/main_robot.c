/**************************************************
* File:  hw6evan.c
* Homework 6 Program
* By Evan Yang
* CMPEN 473, Summer 2025, Penn State University
*
***************************************************/

// header files - at /usr/include and ../include
#include "main_robot.h"

void dly10us(int mult)
{
  int total = 1866*mult;
  int i = 1;
  while(i <= total)     // 10us*mult busy processing
  {
    ++i;
  }
}

void *KeyRead(void * arg)
{
  struct  key_thread_param * param = (struct key_thread_param *)arg;
  struct  thread_command key_cmd = {0, 0};
  int     keyhit = 0;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */
  printf("HW6> ");
  while (!*(param->quit_flag))
  {
    keyhit = get_pressed_key();  // read once every 10ms
    if (keyhit != -1)
    {
      key_cmd.command = keyhit;
      key_cmd.argument = 0;
      if (keyhit == 'q') printf("\n");
      else if (keyhit != 'm') printf("\nHW6> ");
      if (!(FIFO_FULL(param->key_fifo))) {
        FIFO_INSERT(param->key_fifo, key_cmd); // send q command to other threads

      }
      else printf( "key_fifo queue full\nHW6> " );

      if (keyhit ==  'q' ) {
        *param->quit_flag = true;
        // printf("\n");
      }
    }
    wait_period( &timer_state, 10u ); /* 10ms */
  }
  printf( "     KeyRead function done\n" );
  return NULL;
}

void *Control(void * arg)
{
  struct  control_thread_param * param = (struct control_thread_param *)arg;
  struct  thread_command key_cmd = {0, 0};  // copy of input, key_cmd from key_thread
  struct  thread_command filt_cmd = {0, 0};  // copy of output, filt_cmd to filter out bad commands
  bool    m_change = false;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY( param->key_fifo ))) {
      FIFO_REMOVE(param->key_fifo, &key_cmd);  // read once every 10ms
      //printf( "HW4> %s = %c\nHW4> ", param->name, key_cmd.command);

      filt_cmd.command = key_cmd.command;  // other key hit
      filt_cmd.argument = key_cmd.argument; // argument by default is 0
      switch (key_cmd.command) {
        case '1': case '2': // activates when m was pressed before (m_change is true)
          if (m_change) {
            if (!(FIFO_FULL(param->motor_lr_fifo))) FIFO_INSERT( param->motor_lr_fifo, filt_cmd );
            else printf( "motor LR fifo queue full\nHW6> " );
            m_change = false;
          }
          else printf( "wrong command\nHW6> " );
          break;
        case 'w': case 'a': case 's': case 'i': case 'j': case 'd': case 'x': case 'q': case 'o': case 'k': // all valid commands
          if (!m_change) { // true m_change means m was pressed before
            if (!(FIFO_FULL(param->motor_lr_fifo))) FIFO_INSERT( param->motor_lr_fifo, filt_cmd );
            else printf( "motor LR fifo queue full\nHW6> " );
          }
          else {
            printf( "wrong command\nHW6> " );
            m_change = false; // reset m_change
          }
          break;
        case 'm':
          if (!m_change) m_change = true; // only flip m_change when m is pressed first time
          else { 
            printf( "wrong command\nHW6> " ); // else double pressed; wrong cmd
            m_change = false; // reset m_change
          }
          break;
        case 'c': case 'v': case 'b': case 'n':
          if (!(FIFO_FULL(param->cam_main_fifo))) FIFO_INSERT( param->cam_main_fifo, filt_cmd );
          else printf( "Cam Main fifo queue full\nHW6> " );
          break;
        case 10: // enter key 
          printf("HW6> ");
          break;
        default:
          printf( "wrong command\nHW6> " );
      }
    }
    wait_period( &timer_state, 10u ); /* 10ms */
  }
  printf( "     Control function done\n" );
  return NULL;
}

void *IRThread(void * arg)
{
  struct  ir_thread_param * param = (struct ir_thread_param *)arg;
  struct  thread_command mot_cmd = {0, 0};
  int     ir_val;
  int     ir_pin;
  int     ir_average = 0;
  int     average_ctr = 0;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  if (param->name[0] == 'L') ir_pin = 24; // left IR pin
  else ir_pin = 25; // right IR pin

  while (!*(param->quit_flag)) {
    if (*(param->mode) == 2 && !*(param->ir_turn_busy)) { // checks independent of inputs & when not busy
      if ((average_ctr == CHECK_AMOUNT) && (ir_average > (2* CHECK_AMOUNT / 3))) { // white detected on the IR sensor
        if (FIFO_COUNT_FREE(param->left_fifo) > 5 && FIFO_COUNT_FREE(param->right_fifo) > 5) {
          *param->ir_turn_busy = true; // set busy to true, so no direction changes

          mot_cmd.command = 's';
          mot_cmd.type = 's'; // speed type commands
          mot_cmd.argument = 0; // not needed
          FIFO_INSERT(param->left_fifo, mot_cmd );
          FIFO_INSERT(param->right_fifo, mot_cmd );

          mot_cmd.command = 'n'; // signal to not modify
          mot_cmd.type = 'd'; // direction type commands
          if (ir_pin == 24) { // left IR pin
            mot_cmd.argument = 2; // forward is 2
            FIFO_INSERT(param->left_fifo, mot_cmd );
            mot_cmd.argument = 1; // backward is 1
            FIFO_INSERT(param->right_fifo, mot_cmd );
          }
          else { // right IR pin
            mot_cmd.argument = 1; // backward is 1
            FIFO_INSERT(param->left_fifo, mot_cmd );
            mot_cmd.argument = 2; // forward is 2
            FIFO_INSERT(param->right_fifo, mot_cmd );
          }

          mot_cmd.command = 'b'; // signal to do turning and set speed 100
          mot_cmd.type = 's'; // speed type commands
          mot_cmd.argument = 0; // not needed
          FIFO_INSERT(param->left_fifo, mot_cmd );
          FIFO_INSERT(param->right_fifo, mot_cmd );

          mot_cmd.command = 'r'; // signal to revert to original
          mot_cmd.type = 'd'; // direction type commands
          FIFO_INSERT(param->left_fifo, mot_cmd );
          FIFO_INSERT(param->right_fifo, mot_cmd );

          mot_cmd.command = 'g';
          mot_cmd.type = 's'; // speed type commands
          mot_cmd.argument = 0; // not needed
          FIFO_INSERT(param->left_fifo, mot_cmd );
          FIFO_INSERT(param->right_fifo, mot_cmd );

          *param->ir_turn_busy = true; // set busy to true, so no direction changes
        }
        else {
          printf( "\n     LR fifo queue not enough space\nHW4> " );
        }
      }
      // reset average values
      ir_average = 0;
      average_ctr = 0;
    }

    // read IR value every 10ms
    ir_val = GPIO_READ(param->io->gpio, ir_pin); // IR value (0 or positive int)
    if (!ir_val) ir_average++; // 0 IR value is white, else no addition when black
    average_ctr++; // increase counter to check
    wait_period(&timer_state, 10u);
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *CamCtrlThread(void * arg)
{
  struct  cam_ctrl_param * param = (struct cam_ctrl_param *)arg;
  int cam_ctr = CHECK_AMOUNT; // to capture first image before polling for inputs
  static struct image_t image;
  struct video_interface_handle_t * handle = NULL; // handle to the video interfaces
  unsigned char * scaled_raw = (unsigned char *)malloc(IMAGE_SIZE); // allocate memory for scaled data  
  struct pixel_format_RGB * scaled_data = (struct pixel_format_RGB *)scaled_raw;
  struct thread_command filt_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct thread_command cam_cmd = {0, 0};  // copy of output, cam_cmd to send to camera threads
  struct timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  handle = video_interface_open("/dev/video0"); // open the camera device
  video_interface_print_modes(handle); // print the available modes
  if (video_interface_set_mode_auto(handle)) {
    printf( "configured resolution: %zux%zu\nHW6> ", handle->configured_width, handle->configured_height );
    *(param->scaled_width) = handle->configured_width/SCALE_REDUCTION_PER_AXIS; // scale the image to a more agreeable size
    *(param->scaled_height) = handle->configured_height/SCALE_REDUCTION_PER_AXIS; // scaled height    
    // create the window to show the bitmap
    //draw_bitmap_create_window(*(param->argc), param->argv, scaled_width, scaled_height );
  }
  else {
    printf( "failed to configure camera\nHW6> " );
  }

  while (!*(param->quit_flag)) { // && !draw_bitmap_window_closed()) {
    if (cam_ctr == CHECK_AMOUNT) {
      if (video_interface_get_image(handle, &image)) {
        //printf("works \n");
        // scale the image to a more agreeable size
        scale_image_data(
          (struct pixel_format_RGB *)&image,
          handle->configured_height,
          handle->configured_width,
          scaled_data, // set scaled image for other camera threads to see
          SCALE_REDUCTION_PER_AXIS,
          SCALE_REDUCTION_PER_AXIS
        );

        memcpy(param->color_raw, scaled_raw, IMAGE_SIZE);
        memcpy(param->gray_raw, scaled_raw, IMAGE_SIZE);
        memcpy(param->bw_raw, scaled_raw, IMAGE_SIZE);
        memcpy(param->bws_raw, scaled_raw, IMAGE_SIZE);

        // send update to all threads
        cam_cmd.command = 'u';
        if (!(FIFO_FULL(param->cam_color_fifo))) FIFO_INSERT(param->cam_color_fifo, cam_cmd );
        else {
          printf( "Color fifo queue full\nHW6> " );
          goto cam_full_fifo; // skip the rest of the code
        }
        if (!(FIFO_FULL(param->cam_gray_fifo))) FIFO_INSERT(param->cam_gray_fifo, cam_cmd );
        else {
          printf( "Gray fifo queue full\nHW6> " );
          goto cam_full_fifo; // skip the rest of the code
        }
        if (!(FIFO_FULL(param->cam_bw_fifo))) FIFO_INSERT(param->cam_bw_fifo, cam_cmd );
        else {
          printf( "BW fifo queue full\nHW6> " );
          goto cam_full_fifo; // skip the rest of the code
        }
        if (!(FIFO_FULL(param->cam_bw_smol_fifo))) FIFO_INSERT(param->cam_bw_smol_fifo, cam_cmd );
        else {
          printf( "BW Smol fifo queue full\nHW6> " );
          goto cam_full_fifo; // skip the rest of the code
        }
      }
      else {
        printf( "failed to capture image\nHW6> " );
      }
      cam_ctr = 0; // reset camera counter
    }
    cam_ctr++;

    if (!(FIFO_EMPTY(param->cam_main_fifo))) {
      FIFO_REMOVE(param->cam_main_fifo, &filt_cmd);  // read once every 10ms
      //printf( "%s = %c\nHW6> ", param->name, filt_cmd.command);
      cam_cmd.command = filt_cmd.command;
      cam_cmd.argument = filt_cmd.argument;
      switch (filt_cmd.command) {
        case 'c':
          if (!(FIFO_FULL(param->cam_color_fifo))) FIFO_INSERT(param->cam_color_fifo, cam_cmd );
          else printf( "Color fifo queue full\nHW6> " );
          break;
        case 'v':
          if (!(FIFO_FULL(param->cam_gray_fifo))) FIFO_INSERT(param->cam_gray_fifo, cam_cmd );
          else printf( "Gray fifo queue full\nHW6> " );
          break;
        case 'b':
          if (!(FIFO_FULL(param->cam_bw_fifo))) FIFO_INSERT(param->cam_bw_fifo, cam_cmd );
          else printf( "BW fifo queue full\nHW6> " );
          break;
        case 'n':
          if (!(FIFO_FULL(param->cam_bw_smol_fifo))) FIFO_INSERT(param->cam_bw_smol_fifo, cam_cmd );
          else printf( "BW Smol fifo queue full\nHW6> " );
        default:
      }
    }
    cam_full_fifo: wait_period(&timer_state, 10u);
    //printf("period end cam ctrl %d // ", cam_ctr);
  }
  video_interface_close(handle);
  free(scaled_raw);
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *CamColorThread(void * arg)
{
  struct cam_thread_param * param = (struct cam_thread_param *)arg;
  struct pixel_format_RGB * scaled_color_data = (struct pixel_format_RGB *)param->image_raw; // use the image_raw as scaled data
  struct draw_bitmap_multiwindow_handle_t *	handle_GUI_color = NULL;
  struct  thread_command cam_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->fifo))) {
      FIFO_REMOVE(param->fifo, &cam_cmd);  // read once every 10ms
      //printf( "%s = %c\nHW6> ", param->name, cam_cmd.command);

      switch(cam_cmd.command) {
        case 'c': // initialize color image
          if (handle_GUI_color == NULL) { // if the window is not created, create it
            handle_GUI_color = draw_bitmap_create_window(*(param->scaled_width), *(param->scaled_height));
            printf("%p\nHW6> ", handle_GUI_color);
          }
          else {
            draw_bitmap_close_window(handle_GUI_color);
            handle_GUI_color = NULL; // reset the handle
          }
          break;
        case 'u': // updates by processing & showing image if enabled via handle
          if (handle_GUI_color != NULL) {
            int row_mid = *(param->scaled_height) / 2; // middle row coordinate on image array 
            int col_mid = *(param->scaled_width) / 2; // middle col coordinate on image array
            // index of the middle pixel in each row (row index at 0th column found by pix_row * times scaled_height)
            for (int pix_row = 0; pix_row < *(param->scaled_height); pix_row++) {
              int index = pix_row * *(param->scaled_width) + col_mid;
              scaled_color_data[index].R = 0; // set the middle pixel to red
              scaled_color_data[index].G = 255; // set the middle pixel to green
              scaled_color_data[index].B = 0; // set the middle pixel to blue
            }

            // index of the middle pixel in each column (col index at 0th row found by row_mid * times scaled_width)
            for (int pix_col = 0; pix_col < *(param->scaled_width); pix_col++) {
              int index = row_mid * *(param->scaled_width) + pix_col; 
              scaled_color_data[index].R = 0; // set the middle pixel to red
              scaled_color_data[index].G = 255; // set the middle pixel to green
              scaled_color_data[index].B = 0; // set the middle pixel to blue
            }

            // draw the bitmap in the window
            draw_bitmap_display(handle_GUI_color, scaled_color_data);
          }
        default:
      }
    }
    wait_period(&timer_state, 10u);
  }
  if (handle_GUI_color != NULL) {
    draw_bitmap_close_window(handle_GUI_color); // close the window
    handle_GUI_color = NULL; // reset the handle
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *CamGrayThread(void * arg)
{
  struct cam_thread_param * param = (struct cam_thread_param *)arg;
  struct pixel_format_RGB * scaled_gray_data = (struct pixel_format_RGB *)param->image_raw; // use the image_raw as scaled data
  struct draw_bitmap_multiwindow_handle_t *	handle_GUI_gray = NULL;
  struct thread_command cam_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->fifo))) {
      FIFO_REMOVE(param->fifo, &cam_cmd);  // read once every 10ms
      //printf( "%s = %c\nHW6> ", param->name, cam_cmd.command);

      switch(cam_cmd.command) {
        case 'v': // initialize color image
          if (handle_GUI_gray == NULL) { // if the window is not created, create it
            handle_GUI_gray = draw_bitmap_create_window(*(param->scaled_width), *(param->scaled_height));
            printf("%p\nHW6> ", handle_GUI_gray);
          }
          else {
            draw_bitmap_close_window(handle_GUI_gray);
            handle_GUI_gray = NULL; // reset the handle
          }
          break;
        case 'u': // updates by processing & showing image if enabled via handle
          if (handle_GUI_gray != NULL) {
            for (int index = 0; index < IMAGE_SIZE/3; index++) {
              // convert RGB to grayscale by averaging the R, G, B values
              int gray_value = (scaled_gray_data[index].R + scaled_gray_data[index].G + scaled_gray_data[index].B) / 3;
              scaled_gray_data[index].R = gray_value; // set R to gray value
              scaled_gray_data[index].G = gray_value; // set G to gray value
              scaled_gray_data[index].B = gray_value; // set B to gray value
            }

            // draw the bitmap in the window
            draw_bitmap_display(handle_GUI_gray, scaled_gray_data);
          }
        default:
      }
    }
    wait_period(&timer_state, 10u);
  }
  if (handle_GUI_gray != NULL) {
    draw_bitmap_close_window(handle_GUI_gray); // close the window
    handle_GUI_gray = NULL; // reset the handle
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *CamBWThread(void * arg)
{
  struct cam_thread_param * param = (struct cam_thread_param *)arg;
  struct pixel_format_RGB * scaled_BW_data = (struct pixel_format_RGB *)param->image_raw; // use the image_raw as scaled data
  struct draw_bitmap_multiwindow_handle_t *	handle_GUI_BW = NULL;
  struct thread_command cam_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->fifo))) {
      FIFO_REMOVE(param->fifo, &cam_cmd);  // read once every 10ms
      //printf( "%s = %c\nHW6> ", param->name, cam_cmd.command);

      switch(cam_cmd.command) {
        case 'b': // initialize color image
          if (handle_GUI_BW == NULL) { // if the window is not created, create it
            handle_GUI_BW = draw_bitmap_create_window(*(param->scaled_width), *(param->scaled_height));
            printf("%p\nHW6> ", handle_GUI_BW);
          }
          else {
            draw_bitmap_close_window(handle_GUI_BW);
            handle_GUI_BW = NULL; // reset the handle
          }
          break;
        case 'u': // updates by processing & showing image if enabled via handle
          if (handle_GUI_BW != NULL) {
            for (int index = 0; index < IMAGE_SIZE/3; index++) {
              // convert RGB to grayscale by averaging the R, G, B values
              int gray_value = (scaled_BW_data[index].R + scaled_BW_data[index].G + scaled_BW_data[index].B) / 3;
              if (gray_value > 127) { // flip to white
                scaled_BW_data[index].R = 255;
                scaled_BW_data[index].G = 255;
                scaled_BW_data[index].B = 255;
              }
              else {
                scaled_BW_data[index].R = 0;
                scaled_BW_data[index].G = 0;
                scaled_BW_data[index].B = 0;
              }
            }

            // draw the bitmap in the window
            draw_bitmap_display(handle_GUI_BW, scaled_BW_data);
          }
        default:
      }
    }
    wait_period(&timer_state, 10u);
  }
  if (handle_GUI_BW != NULL) {
    draw_bitmap_close_window(handle_GUI_BW); // close the window
    handle_GUI_BW = NULL; // reset the handle
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *CamBWSThread(void * arg)
{
  struct cam_thread_param * param = (struct cam_thread_param *)arg;
  unsigned char * scaled_smol_raw = (unsigned char *)malloc(SMOL_IMAGE_SIZE); // allocate memory for scaled data  
  struct pixel_format_RGB * scaled_BWS_data = (struct pixel_format_RGB *)scaled_smol_raw; // use the image_raw as scaled data
  struct draw_bitmap_multiwindow_handle_t *	handle_GUI_BWS = NULL;
  struct thread_command cam_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag)) {
    if (!(FIFO_EMPTY(param->fifo))) {
      FIFO_REMOVE(param->fifo, &cam_cmd);  // read once every 10ms
      //printf( "%s = %c\nHW6> ", param->name, cam_cmd.command);

      switch(cam_cmd.command) {
        case 'n': // initialize color image
          if (handle_GUI_BWS == NULL) { // if the window is not created, create it
            handle_GUI_BWS = draw_bitmap_create_window(*(param->scaled_width) / SMALL_SCALE, *(param->scaled_height) / SMALL_SCALE);
            printf("%p\nHW6> ", handle_GUI_BWS);
          }
          else {
            draw_bitmap_close_window(handle_GUI_BWS);
            handle_GUI_BWS = NULL; // reset the handle
          }
          break;
        case 'u': // updates by processing & showing image if enabled via handle
          if (handle_GUI_BWS != NULL) {
            scale_image_data(
              (struct pixel_format_RGB *)param->image_raw,
              *(param->scaled_height),
              *(param->scaled_width),
              scaled_BWS_data, // set scaled image for other camera threads to see
              SMALL_SCALE,
              SMALL_SCALE
            );
            for (int index = 0; index < SMOL_IMAGE_SIZE/3; index++) {
              // convert RGB to grayscale by averaging the R, G, B values
              int gray_value = (scaled_BWS_data[index].R + scaled_BWS_data[index].G + scaled_BWS_data[index].B) / 3;
              if (gray_value > 127) { // flip to white
                scaled_BWS_data[index].R = 255;
                scaled_BWS_data[index].G = 255;
                scaled_BWS_data[index].B = 255;
              }
              else {
                scaled_BWS_data[index].R = 0;
                scaled_BWS_data[index].G = 0;
                scaled_BWS_data[index].B = 0;
              }
            }

            // draw the bitmap in the window
            draw_bitmap_display(handle_GUI_BWS, scaled_BWS_data);
          }
        default:
      }
    }
    wait_period(&timer_state, 10u);
  }
  if (handle_GUI_BWS != NULL) {
    draw_bitmap_close_window(handle_GUI_BWS); // close the window
    handle_GUI_BWS = NULL; // reset the handle
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *MotorLRThread(void *arg)
{
  struct  motor_lr_ctrl_param * param = (struct motor_lr_ctrl_param *)arg;
  struct  thread_command filt_cmd = {0, 0};  // copy of input, filt_cmd from control_thread
  struct  thread_command mot_cmd = {0, 0};  // copy of output, mot_cmd to sed to motor threads
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait, ie. set interrupt
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  while (!*(param->quit_flag))
  {
    if (!(FIFO_EMPTY(param->motor_lr_fifo))) {
      FIFO_REMOVE(param->motor_lr_fifo, &filt_cmd);  // read once every 10ms
      //printf( "%s = %c %d\nHW4> ", param->name, filt_cmd.command, param->mode);

      if (filt_cmd.command == '1' || filt_cmd.command == '2') {
        *(param->mode) = filt_cmd.command - '0'; // check if mode change input (if not, check the current mode)
        if (*(param->mode) == 2) { // set speeds and turning
          if (FIFO_COUNT_FREE(param->left_fifo) > 3 && FIFO_COUNT_FREE(param->right_fifo) > 3) {
            mot_cmd.type = 's'; // speed type commands
            mot_cmd.command = 'k'; // decrease to minimum
            mot_cmd.argument = 90; // decrease turning angle to 5 degrees
            FIFO_INSERT(param->left_fifo, mot_cmd);
            FIFO_INSERT(param->right_fifo, mot_cmd);

            mot_cmd.type = 's'; // speed type commands
            mot_cmd.command = 'o'; // increase to maximum
            mot_cmd.argument = 10; // increase turning angle to 15
            FIFO_INSERT(param->left_fifo, mot_cmd);
            FIFO_INSERT(param->right_fifo, mot_cmd);

            mot_cmd.type = 's'; // speed type commands
            mot_cmd.command = 'i'; // increase to maximum
            mot_cmd.argument = 100; // set to max speed
            FIFO_INSERT(param->left_fifo, mot_cmd);
            FIFO_INSERT(param->right_fifo, mot_cmd);
          }
          else {
            printf( "LR speed fifo queue full\nHW6> " );
          }
        }
      }

      else if (*(param->mode) == 1) {
        switch (filt_cmd.command) {
          case 's': // stop
            if (FIFO_COUNT_FREE(param->left_fifo) > 2 && FIFO_COUNT_FREE(param->right_fifo) > 2) {
              mot_cmd.command = filt_cmd.command;
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = filt_cmd.command;
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 0; // stop is 0
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else printf( "LR fifo queue not enough space\nHW6> " );
            break;
          case 'w': // go forward (stop, then go)
            if (FIFO_COUNT_FREE(param->left_fifo) > 3 && FIFO_COUNT_FREE(param->right_fifo) > 3) {
              mot_cmd.command = 's'; // signal to save direction & stop speed
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 's'; // signal to modify
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 2; // forward is 2
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'g'; // signal to start moving using previous speed
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else printf( "LR fifo queue not enough space\nHW6> " );
            break;

          case 'x': // go backward (stop, then go) but does not work in m2
            if (FIFO_COUNT_FREE(param->left_fifo) > 3 && FIFO_COUNT_FREE(param->right_fifo) > 3) {
              mot_cmd.command = 's';// signal to save direction and stop speed
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 's'; // signal to modify
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 1; // backward is 1
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'g'; // signal to start moving using previous speed
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else printf( "LR fifo queue not enough space\nHW6> " );
            break;

          case 'i': case 'j': case 'o': case 'k': // does not accept input in m2
            mot_cmd.command = filt_cmd.command; // copy command
            mot_cmd.type = 's'; // speed type commands
            mot_cmd.argument = 5; // percentage of increase/decrease
            if (!(FIFO_FULL(param->left_fifo)) && !(FIFO_FULL(param->right_fifo))) {
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else printf( "LR speed fifo queue full\nHW6> " );
            break;

          case 'a': case 'd':// left turn, right turn (m2 no input)
            if (FIFO_COUNT_FREE(param->left_fifo) > 5 && FIFO_COUNT_FREE(param->right_fifo) > 5) {
              mot_cmd.command = 's';
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'n'; // signal to not modify
              mot_cmd.type = 'd'; // direction type commands
              if (filt_cmd.command == 'a') { // left turn (a)
                mot_cmd.argument = 2; // forward is 2
                FIFO_INSERT(param->left_fifo, mot_cmd );
                mot_cmd.argument = 1; // backward is 1
                FIFO_INSERT(param->right_fifo, mot_cmd );
              }
              else { // right turn (d)
                mot_cmd.argument = 1; // backward is 1
                FIFO_INSERT(param->left_fifo, mot_cmd );
                mot_cmd.argument = 2; // forward is 2
                FIFO_INSERT(param->right_fifo, mot_cmd );
              }

              mot_cmd.command = 'b'; // signal to do turning and set speed 100
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'r'; // signal to not modify
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 3; // revert to original
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'g';
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else printf( "LR fifo queue not enough space\nHW6> " );
            break;
          default: // nothing happens (commands already filtered out)
        }
      }

      else if (*(param->mode) == 2) {
        switch (mot_cmd.command) {
          case 's': // stop
            if (FIFO_COUNT_FREE(param->left_fifo) > 2 && FIFO_COUNT_FREE(param->right_fifo) > 2) {
              mot_cmd.command = filt_cmd.command;
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = filt_cmd.command;
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 0; // stop is 0
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else {
              printf( "LR fifo queue not enough space\nHW6> " );
            }
            break;
          case 'w': // go forward
            if (FIFO_COUNT_FREE(param->left_fifo) > 3 && FIFO_COUNT_FREE(param->right_fifo) > 3) {
              mot_cmd.command = 's'; // signal to save direction & stop speed
              mot_cmd.type = 'd'; // direction type commands
              mot_cmd.argument = 2; // forward is 2
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );

              mot_cmd.command = 'g'; // signal to start moving using previous speed
              mot_cmd.type = 's'; // speed type commands
              mot_cmd.argument = 0; // not needed
              FIFO_INSERT(param->left_fifo, mot_cmd );
              FIFO_INSERT(param->right_fifo, mot_cmd );
            }
            else {
              printf( "LR fifo queue not enough space\nHW6> " );
            }
          default:
        }
      }
    }
    wait_period( &timer_state, 10u ); /* 10ms */
  }
  printf( "     Motor LR function done\n" );
  return NULL;
}

void *MotorSpdThread(void * arg)
{
  struct  motor_thread_param * param = (struct motor_thread_param *)arg;
  struct  thread_command mot_cmd = {0, 0};
  bool    busy = false;
  int     speed = 0; // assume init is 0
  int     busy_time = 15;
  struct  timespec  timer_state;
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  int mot_pin;
  if (param->name[0] == 'L') { // left motor pin
    mot_pin = 12;
  }
  else { // right motor pin
    mot_pin = 13;
  }

  while (!*(param->quit_flag)) {
    //printf("%d ", GPIO_READ(param->io->gpio, 24));
    int temp_busy;
    if (!busy) {
      if (!(FIFO_EMPTY( param->fifo )) && param->fifo->data[param->fifo->next_remove].type == 's') {
        FIFO_REMOVE( param->fifo, &mot_cmd );  // read once every 10ms
        //printf( "%s = %d  %c pin %d speed %d angle %d\nHW4> ", param->name, mot_cmd.argument, mot_cmd.command, mot_pin, speed, busy_time);

        char cmd_type = mot_cmd.command;
        int cmd_arg = mot_cmd.argument;

        switch (cmd_type) {
          case 'b': // busy for time set in cmd_arg
            busy = true;
            *param->turn_busy = true;
            temp_busy = busy_time;
            if (mot_pin == 12) param->io->pwm->DAT1 = 100; // left
            else param->io->pwm->DAT2 = 100; // right
            break;
          case 's': // stop moving but save last running speed
            dly10us(1000); // ramps in 0.01 second intervalse
            GPIO_CLR(param->io->gpio, mot_pin); // stops motor completely
            break;
          case 'g': // continue going based on last saved speed
            dly10us(1000); // ramps in 0.01 second intervals
            if (mot_pin == 12) param->io->pwm->DAT1 = speed; // left
            else param->io->pwm->DAT2 = speed; // right
            break;
          case 'i': // increase speed by 5%
            if (speed <= (100 - cmd_arg)) speed += cmd_arg; // does not exceed 100% (init max 95%)
            else speed = 100;
            if (mot_pin == 12) param->io->pwm->DAT1 = speed; // left
            else param->io->pwm->DAT2 = speed; // right
            break;
          case 'j': // decrease speed by 5%
            if (speed >= cmd_arg) speed -= cmd_arg; // does not dip below 0% (init min 5%)
            else speed = 0;
            if (mot_pin == 12) param->io->pwm->DAT1 = speed; // left
            else param->io->pwm->DAT2 = speed; // right
            break;
          case 'o': // increase turn delay by 5 degrees
            if (busy_time <= (90 - cmd_arg)) busy_time += cmd_arg;
            else busy_time = 90;
            break;
          case 'k': // decrease turn delay by 5 degrees
            if (busy_time >= (5 + cmd_arg)) busy_time -= cmd_arg;
            else busy_time = 5;
            break;
          default:
        }
      }
    }
    else {
      if (temp_busy != 0) temp_busy--;
      else {
        busy = false;
        *param->turn_busy = false;
      }
    }
    wait_period( &timer_state, 10u ); /* 10ms */
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

void *MotorDirThread(void * arg)
{
  struct  motor_thread_param * param = (struct motor_thread_param *)arg;
  struct  thread_command mot_cmd = {0, 0};
  int     dir = 0; // direction default is nothing
  struct  timespec  timer_state; 
             // used to wake up every 10ms with wait_period() function,
             // similar to interrupt occuring every 10ms

  // start 10ms timed wait
  wait_period_initialize( &timer_state );
  wait_period( &timer_state, 10u ); /* 10ms */

  int mot_pin1, mot_pin2;
  if (param->name[0] == 'L') { // left motor pins
    mot_pin1 = 5;
    mot_pin2 = 6;
  }
  else { // right motor pins
    mot_pin1 = 22;
    mot_pin2 = 23;
  }

  while (!*(param->quit_flag)) {
    if (!(*param->turn_busy)) {
      if (!(FIFO_EMPTY( param->fifo )) && param->fifo->data[param->fifo->next_remove].type == 'd') {
        FIFO_REMOVE( param->fifo, &mot_cmd );  // read once every 10ms
        //printf( "%s = %d  %c pins %d %d dir %d\nHW4> ", param->name, mot_cmd.argument, mot_cmd.command, mot_pin1, mot_pin2, dir);        
        
        char cmd_type = mot_cmd.command;
        int cmd_arg = mot_cmd.argument;
        switch (cmd_type) {
          case 's':
            dir = cmd_arg;
            break;
          case 'n':
            *param->turn_busy = true; // n signals a turn, so set busy
          case 'r':
            cmd_arg = dir; // r uses current dir to revert to original (will NOT happen in s cases)
            break;
          default:
        }
        switch (cmd_arg) {
          case 0: // stop (0,0)
            GPIO_CLR(param->io->gpio, mot_pin1);
            GPIO_CLR(param->io->gpio, mot_pin2);
            break;          
          case 1: // backward (1,0)
            GPIO_SET(param->io->gpio, mot_pin1);
            GPIO_CLR(param->io->gpio, mot_pin2);
            break;   
          case 2: // forward (0,1)
            GPIO_CLR(param->io->gpio, mot_pin1);
            GPIO_SET(param->io->gpio, mot_pin2);
            break;   
          default:
        }
      }
    }
    wait_period( &timer_state, 10u ); /* 10ms */
  }
  printf( "     %s function done\n", param->name );
  return NULL;
}

int main(int argc, char * argv[])
{
  struct io_peripherals *io;
  bool quit_flag =    false;
  bool ir_turn_busy = false;
  bool turn_busy_l =  false;
  bool turn_busy_r =  false;
  int mode = 1;
  unsigned int scaled_width = 0;
  unsigned int scaled_height = 0;
  unsigned char * color_raw = (unsigned char *)malloc(IMAGE_SIZE);
  unsigned char * gray_raw = (unsigned char *)malloc(IMAGE_SIZE);
  unsigned char * bw_raw = (unsigned char *)malloc(IMAGE_SIZE);
  unsigned char * bws_raw = (unsigned char *)malloc(IMAGE_SIZE);
  pthread_t tmls;
  pthread_t tmrs;
  pthread_t tmld;
  pthread_t tmrd;
  pthread_t tk;
  pthread_t tc;
  pthread_t tlir;
  pthread_t trir;
  pthread_t tcamcol;
  pthread_t tcamgray;
  pthread_t tcambw;
  pthread_t tcambws;
  pthread_t tcam;
  pthread_t tmlr;
  struct fifo_t key_fifo =         {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t motor_lr_fifo =    {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t left_fifo =        {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t right_fifo =       {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t cam_main_fifo =    {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t cam_color_fifo =   {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t cam_gray_fifo =    {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t cam_bw_fifo =      {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct fifo_t cam_bw_smol_fifo = {{}, 0, 0, PTHREAD_MUTEX_INITIALIZER};
  struct motor_thread_param  lspd_param =     {"LSpd", NULL, &left_fifo, &turn_busy_l, &quit_flag};
  struct motor_thread_param  rspd_param =     {"RSpd", NULL, &right_fifo, &turn_busy_r, &quit_flag};
  struct motor_thread_param  ldir_param =     {"LDir", NULL, &left_fifo, &turn_busy_l, &quit_flag};
  struct motor_thread_param  rdir_param =     {"RDir", NULL, &right_fifo, &turn_busy_r, &quit_flag};
  struct key_thread_param key_param =         {"Key", &key_fifo, &quit_flag};
  struct control_thread_param con_param =     {"Con", &key_fifo, &motor_lr_fifo, &cam_main_fifo, &quit_flag};
  struct ir_thread_param lir_param =          {"LIR", NULL, &left_fifo, &right_fifo, &mode, &ir_turn_busy, &quit_flag};
  struct ir_thread_param rir_param =          {"RIR", NULL, &left_fifo, &right_fifo, &mode, &ir_turn_busy, &quit_flag};
  struct cam_ctrl_param cam_param =           {"CamCtrl", &cam_main_fifo, &cam_color_fifo, &cam_gray_fifo, &cam_bw_fifo, &cam_bw_smol_fifo, color_raw, gray_raw, bw_raw, bws_raw, &scaled_width, &scaled_height, &quit_flag};
  struct cam_thread_param cam_color_param =   {"CamColor", &cam_color_fifo, color_raw, &scaled_width, &scaled_height, &quit_flag};
  struct cam_thread_param cam_gray_param =    {"CamGray", &cam_gray_fifo, gray_raw, &scaled_width, &scaled_height, &quit_flag};
  struct cam_thread_param cam_bw_param =      {"CamBW", &cam_bw_fifo, bw_raw, &scaled_width, &scaled_height, &quit_flag};
  struct cam_thread_param cam_bw_smol_param = {"CamBWSmol", &cam_bw_smol_fifo, bws_raw, &scaled_width, &scaled_height, &quit_flag};
  struct motor_lr_ctrl_param mlr_param  =     {"MotLR", &motor_lr_fifo, &left_fifo, &right_fifo, &mode, &quit_flag};

  io = import_registers();
  if (io != NULL) {
    /* print where the I/O memory was actually mapped to */
    printf( "mem at 0x%8.8X\n", (unsigned int)io );

    io->gpio->GPFSEL1.field.FSEL2 = GPFSEL_ALTERNATE_FUNCTION0; //set GPIO 12 as alt func (left PWM)
    io->gpio->GPFSEL1.field.FSEL3 = GPFSEL_ALTERNATE_FUNCTION0; //set GPIO 13 as alt func (right PWM)
    io->gpio->GPFSEL0.field.FSEL5 = GPFSEL_OUTPUT;   //set GPIO 05 as output (left motor 1)
    io->gpio->GPFSEL0.field.FSEL6 = GPFSEL_OUTPUT;   //set GPIO 06 as output (left motor 2)
    io->gpio->GPFSEL2.field.FSEL2 = GPFSEL_OUTPUT;   //set GPIO 22 as output (right motor 1)
    io->gpio->GPFSEL2.field.FSEL3 = GPFSEL_OUTPUT;   //set GPIO 23 as output (right motor 2)
    io->gpio->GPFSEL2.field.FSEL4 = GPFSEL_INPUT;    //set GPIO 24 as input (left IR sensor)
    io->gpio->GPFSEL2.field.FSEL5 = GPFSEL_INPUT;    //set GPIO 25 as input (right IR sensor)

    /* configure the PWM channels */
    enable_pwm_clock(io->cm, io->pwm);  /* Hardware pwm needs clock to work */
    io->pwm->RNG1 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm->RNG2 = PWM_RANGE;     /* the range value, 100 level steps */
    io->pwm->DAT1 = 1;             /* initial beginning level=1/100=1% */
    io->pwm->DAT2 = 1;             /* initial beginning level=1/100=1% */
    io->pwm->CTL.field.MODE1 = 0;  /* PWM mode */
    io->pwm->CTL.field.MODE2 = 0;  /* PWM mode */
    io->pwm->CTL.field.RPTL1 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.RPTL2 = 1;  /* not using FIFO, but repeat the last byte anyway */
    io->pwm->CTL.field.SBIT1 = 0;  /* idle low */
    io->pwm->CTL.field.SBIT2 = 0;  /* idle low */
    io->pwm->CTL.field.POLA1 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.POLA2 = 0;  /* non-inverted polarity */
    io->pwm->CTL.field.USEF1 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.USEF2 = 0;  /* do not use FIFO */
    io->pwm->CTL.field.MSEN1 = 1;  /* use M/S algorithm, level=pwm->DAT1/PWM_RANGE */
    io->pwm->CTL.field.MSEN2 = 1;  /* use M/S algorithm, level=pwm->DAT2/PWM_RANGE */
    io->pwm->CTL.field.CLRF1 = 1;  /* clear the FIFO, even though it is not used */
    io->pwm->CTL.field.PWEN1 = 1;  /* enable the PWM channel */
    io->pwm->CTL.field.PWEN2 = 1;  /* enable the PWM channel */

    lspd_param.io = io;
    rspd_param.io = io;
    ldir_param.io = io;
    rdir_param.io = io;
    lir_param.io = io;
    rir_param.io = io;

    GPIO_SET(io->gpio, 12);   //set left  motor PWM to 100%
    GPIO_SET(io->gpio, 13);   //set right motor PWM to 100%
    GPIO_CLR(io->gpio, 05);   //stop left motor
    GPIO_CLR(io->gpio, 06);   //  0,0=GPIO05,GPIO06
    GPIO_CLR(io->gpio, 22);   //stop right motor
    GPIO_CLR(io->gpio, 23);   //  0,0=GPIO22,GPIO23

    // Start bitmap
    draw_bitmap_start(argc, argv);

    // Create threads and run them in parallel
    pthread_create(&tmls, NULL, MotorSpdThread, (void *)&lspd_param);
    pthread_create(&tmrs, NULL, MotorSpdThread, (void *)&rspd_param);
    pthread_create(&tmld, NULL, MotorDirThread, (void *)&ldir_param);
    pthread_create(&tmrd, NULL, MotorDirThread, (void *)&rdir_param);
    pthread_create(&tk, NULL, KeyRead, (void *)&key_param);
    pthread_create(&tc, NULL, Control, (void *)&con_param);
    pthread_create(&tlir, NULL, IRThread, (void *)&lir_param);
    pthread_create(&trir, NULL, IRThread, (void *)&rir_param);
    pthread_create(&tcam, NULL, CamCtrlThread, (void *)&cam_param);
    pthread_create(&tcamcol, NULL, CamColorThread, (void *)&cam_color_param);
    pthread_create(&tcamgray, NULL, CamGrayThread, (void *)&cam_gray_param);
    pthread_create(&tcambw, NULL, CamBWThread, (void *)&cam_bw_param);
    pthread_create(&tcambws, NULL, CamBWSThread, (void *)&cam_bw_smol_param);
    pthread_create(&tmlr, NULL, MotorLRThread, (void *)&mlr_param);

    // Wait to finish all threads
    pthread_join(tmls, NULL);
    pthread_join(tmrs, NULL);
    pthread_join(tmld, NULL);
    pthread_join(tmrd, NULL);
    pthread_join(tk, NULL);
    pthread_join(tc, NULL);
    pthread_join(tlir, NULL);
    pthread_join(trir, NULL);
    pthread_join(tcam, NULL);
    pthread_join(tcamcol, NULL);
    pthread_join(tcamgray, NULL);
    pthread_join(tcambw, NULL);
    pthread_join(tcambws, NULL);
    pthread_join(tmlr, NULL);

    /* main task finished  */
    /* clean the GPIO pins */
    GPIO_CLR(io->gpio, 12);   //set left  motor PWM to 0%
    GPIO_CLR(io->gpio, 13);   //set right motor PWM to 0%
    GPIO_CLR(io->gpio, 05);   //stop left motor
    GPIO_CLR(io->gpio, 06);   //  0,0=GPIO05,GPIO06
    GPIO_CLR(io->gpio, 22);   //stop right motor
    GPIO_CLR(io->gpio, 23);   //  0,0=GPIO22,GPIO23

    /* close camera */
    draw_bitmap_stop();
    free(color_raw);
    free(gray_raw);
    free(bw_raw);
    free(bws_raw);
  }
  else ; /* warning message already issued */

  printf("     Main program done\n \n");
  return 0;
}
