TARGET=main_robot

SOURCES=../include/import_registers.c \
        ../include/enable_pwm_clock.c \
        ../include/wait_period.c \
	../include/keypress.c \
	../include/video_interface.c \
	../include/wait_key.c \
	../include/scale_image_data.c \
	../include/draw_bitmap_multiwindow.c \
        main_robot.c

OBJECTS=$(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(notdir $(SOURCES))))

CFLAGS = `pkg-config --cflags --libs gtk+-2.0` \
         -Wall \
         -Werror \
         -Wno-deprecated-declarations \
         -g

all: $(TARGET)

#all: $(OBJECTS)
#	g++ $^ $(CFLAGS) $(OBJECTS) -lm -lpthread -lswscale -o $(TARGET)

$(TARGET): $(OBJECTS)
	gcc $^ $(CFLAGS) -o $@ -lm -lpthread -lswscale

clean:
	rm -f $(OBJECTS) $(TARGET)

%.o:%.c
	gcc -c $< $(CFLAGS) -o $@

%.o:../include/%.c
	gcc $(CFLAGS) -c $< -o $@
