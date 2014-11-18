#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <AxisAlly.h>

#include "SDL.h"

//----------------------------------------------------------

// A set of very useful macros that you will find in most
// code that I write whether I use them in a program or
// not.

#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define abs(a) (((a)<0) ? -(a) : (a))
#define sign(a) (((a)<0) ? -1 : (a)>0 ? 1 : 0)

//----------------------------------------------------------

// The following code implements a Bresenham line drawing
// algorithm. There are 4 separate routines each optimized
// for one of the four pixel depths supported by SDL. SDL
// support many pixel formats, but it only support 8, 16,
// 24, and 32 bit pixels.

//----------------------------------------------------------

// Draw lines in 8 bit surfaces.

static void line8(SDL_Surface *s, 
                  int x1, int y1, 
                  int x2, int y2, 
                  Uint32 color)
{
  int d;
  int x;
  int y;
  int ax;
  int ay;
  int sx;
  int sy;
  int dx;
  int dy;

  Uint8 *lineAddr;
  Sint32 yOffset;

  dx = x2 - x1;  
  ax = abs(dx) << 1;  
  sx = sign(dx);

  dy = y2 - y1;  
  ay = abs(dy) << 1;  
  sy = sign(dy);
  yOffset = sy * s->pitch;

  x = x1;
  y = y1;

  lineAddr = ((Uint8 *)(s->pixels)) + (y * s->pitch);
  if (ax>ay)
  {                      /* x dominant */
    d = ay - (ax >> 1);
    for (;;)
    {
      *(lineAddr + x) = (Uint8)color;

      if (x == x2)
      {
        return;
      }
      if (d>=0)
      {
        y += sy;
        lineAddr += yOffset;
        d -= ax;
      }
      x += sx;
      d += ay;
    }
  }
  else
  {                      /* y dominant */
    d = ax - (ay >> 1);
    for (;;)
    {
      *(lineAddr + x) = (Uint8)color;

      if (y == y2)
      {
        return;
      }
      if (d>=0) 
      {
        x += sx;
        d -= ay;
      }
      y += sy;
      lineAddr += yOffset;
      d += ax;
    }
  }
}

//----------------------------------------------------------

// Draw lines in 16 bit surfaces. Note that this code will
// also work on 15 bit surfaces.

static void line16(SDL_Surface *s, 
                   int x1, int y1, 
                   int x2, int y2, 
                   Uint32 color)
{
  int d;
  int x;
  int y;
  int ax;
  int ay;
  int sx;
  int sy;
  int dx;
  int dy;

  Uint8 *lineAddr;
  Sint32 yOffset;

  dx = x2 - x1;  
  ax = abs(dx) << 1;  
  sx = sign(dx);

  dy = y2 - y1;  
  ay = abs(dy) << 1;  
  sy = sign(dy);
  yOffset = sy * s->pitch;

  x = x1;
  y = y1;

  lineAddr = ((Uint8 *)s->pixels) + (y * s->pitch);
  if (ax>ay)
  {                      /* x dominant */
    d = ay - (ax >> 1);
    for (;;)
    {
      *((Uint16 *)(lineAddr + (x << 1))) = (Uint16)color;

      if (x == x2)
      {
        return;
      }
      if (d>=0)
      {
        y += sy;
        lineAddr += yOffset;
        d -= ax;
      }
      x += sx;
      d += ay;
    }
  }
  else
  {                      /* y dominant */
    d = ax - (ay >> 1);
    for (;;)
    {
      *((Uint16 *)(lineAddr + (x << 1))) = (Uint16)color;

      if (y == y2)
      {
        return;
      }
      if (d>=0) 
      {
        x += sx;
        d -= ay;
      }
      y += sy;
      lineAddr += yOffset;
      d += ax;
    }
  }
}

//----------------------------------------------------------

// Draw lines in 24 bit surfaces. 24 bit surfaces require
// special handling because the pixels don't fall on even
// address boundaries. Instead of being able to store a
// single byte, word, or long you have to store 3
// individual bytes. As a result 24 bit graphics is slower
// than the other pixel sizes.

static void line24(SDL_Surface *s, 
                   int x1, int y1, 
                   int x2, int y2, 
                   Uint32 color)
{
  int d;
  int x;
  int y;
  int ax;
  int ay;
  int sx;
  int sy;
  int dx;
  int dy;

  Uint8 *lineAddr;
  Sint32 yOffset;

#if (SDL_BYTEORDER == SDL_BIG_ENDIAN)
  color <<= 8;
#endif

  dx = x2 - x1;  
  ax = abs(dx) << 1;  
  sx = sign(dx);

  dy = y2 - y1;  
  ay = abs(dy) << 1;  
  sy = sign(dy);
  yOffset = sy * s->pitch;

  x = x1;
  y = y1;

  lineAddr = ((Uint8 *)(s->pixels)) + (y * s->pitch);
  if (ax>ay)
  {                      /* x dominant */
    d = ay - (ax >> 1);
    for (;;)
    {
      Uint8 *p = (lineAddr + (x * 3));
      memcpy(p, &color, 3);

      if (x == x2)
      {
        return;
      }
      if (d>=0)
      {
        y += sy;
        lineAddr += yOffset;
        d -= ax;
      }
      x += sx;
      d += ay;
    }
  }
  else
  {                      /* y dominant */
    d = ax - (ay >> 1);
    for (;;)
    {
      Uint8 *p = (lineAddr + (x * 3));
      memcpy(p, &color, 3);

      if (y == y2)
      {
        return;
      }
      if (d>=0) 
      {
        x += sx;
        d -= ay;
      }
      y += sy;
      lineAddr += yOffset;
      d += ax;
    }
  }
}

//----------------------------------------------------------

// Draw lines in 32 bit surfaces. Note that this routine
// ignores alpha values. It writes them into the surface
// if they are included in the pixel, but does nothing
// else with them.

static void line32(SDL_Surface *s, 
                   int x1, int y1, 
                   int x2, int y2, 
                   Uint32 color)
{
  int d;
  int x;
  int y;
  int ax;
  int ay;
  int sx;
  int sy;
  int dx;
  int dy;

  Uint8 *lineAddr;
  Sint32 yOffset;

  dx = x2 - x1;  
  ax = abs(dx) << 1;  
  sx = sign(dx);

  dy = y2 - y1;  
  ay = abs(dy) << 1;  
  sy = sign(dy);
  yOffset = sy * s->pitch;

  x = x1;
  y = y1;

  lineAddr = ((Uint8 *)(s->pixels)) + (y * s->pitch);
  if (ax>ay)
  {                      /* x dominant */
    d = ay - (ax >> 1);
    for (;;)
    {
      *((Uint32 *)(lineAddr + (x << 2))) = (Uint32)color;

      if (x == x2)
      {
        return;
      }
      if (d>=0)
      {
        y += sy;
        lineAddr += yOffset;
        d -= ax;
      }
      x += sx;
      d += ay;
    }
  }
  else
  {                      /* y dominant */
    d = ax - (ay >> 1);
    for (;;)
    {
      *((Uint32 *)(lineAddr + (x << 2))) = (Uint32)color;

      if (y == y2)
      {
        return;
      }
      if (d>=0) 
      {
        x += sx;
        d -= ay;
      }
      y += sy;
      lineAddr += yOffset;
      d += ax;
    }
  }
}

//----------------------------------------------------------

// Examine the depth of a surface and select a line
// drawing routine optimized for the bytes/pixel of the
// surface.

static void line(SDL_Surface *s, 
                 int x1, int y1, 
                 int x2, int y2, 
                 Uint32 color)
{
  switch (s->format->BytesPerPixel)
  {
  case 1:
    line8(s, x1, y1, x2, y2, color);
    break;
  case 2:
    line16(s, x1, y1, x2, y2, color);
    break;
  case 3:
    line24(s, x1, y1, x2, y2, color);
    break;
  case 4:
    line32(s, x1, y1, x2, y2, color);
    break;
  }
}

//----------------------------------------------------------

// sweepLine animates a line on a surface based on the
// elapsed time.

class simAxis
{
private:
  AxisAlly *axis;
  SDL_Surface *s;             // The surface to draw on.
  Uint32 red, green, blue;    // The color of the line.
  int last;                   // last time update() was
                              // called.
  int maxx;                   // Maximum valid X value.
  int maxy;                   // Maximum valid Y value.
  int desired;
  float last_velocity;

public:

  // sweepLine animates a line on a surface. It is
  // initialized with a pointer to the surface to draw the
  // line on, a pixel value that specifies the color of
  // the line, the current time, and the initial locations
  // of the line end points and their
  // velocities. Velocities are specified in
  // pixels/millisecond.

  // This method initializes the class and forces the end
  // points of the lines to be inside the boundaries of
  // the surface. If it didn't do that the line drawing
  // code would try to write outside of the surface and
  // crash the program.

  simAxis(SDL_Surface *s, 
            float maxv, float maxa,
            Uint32 red, Uint32 green, Uint32 blue,
            int time) :
    s(s),
    red(red), green(green), blue(blue),
    last(time)
  {

    axis = new AxisAlly_Sim();

    // Set the values of maxx and maxy to one less than
    // the width and height. Do this makes clipping easier
    // to code.

    maxx = 0;
    maxy = 0;

    if (NULL != s)
    {
      maxx = s->w - 1;
      maxy = s->h - 1;
    }

    axis->setLocationRange(0, maxx);

    desired = maxx/2;

    axis->setVelocityMax(maxv);
    axis->setAccelerationMax(maxa);
    axis->setLocation(desired);
    axis->begin();

    axis->moveLocation(desired);

    last_velocity = 0;
  }

  ~simAxis() { delete axis; }

  void update(long now)
  {
    int dt = now - last;
    int location;
    float velocity, vmax, amax, accel;
    last = now;

    if (!axis->update(dt)) {
        desired = maxx * drand48();
        axis->moveLocation(desired);
    }

    location = axis->getLocation();
    velocity = fabsf(axis->getVelocity());
    vmax = axis->getVelocityMax();
    amax = axis->getAccelerationMax();

    accel = fabsf((velocity - last_velocity) / dt * 1000.0);

    // Draw the yaxis
    line(s, 0, maxy/2, maxx, maxy/2, blue);

    // Draw the desired position
    line(s, desired, maxy * 0.4, desired, maxy * 0.6, green);

    // Draw the velocity limit endcap
    line(s, location - 10, maxy/2 - vmax, location + 10, maxy/2 - vmax, red);

    // Draw the acceleration limit endcap
    line(s, location - 10, maxy/2 + amax, location + 10, maxy/2 + amax, red);

    // Draw the location line
    line(s, location, maxy/2 - velocity, location, maxy/2 + accel, red);

    last_velocity = velocity;
  }

};

//----------------------------------------------------------

// gameTime keeps track of game time as opposed to real
// time. Game time can start and stop and even change its
// speed while real time just keeps ticking along.

class gameTime
{
private:
  int startTime;              // Last time the clock was
                              // started.
  int baseTime;               // How much game time passed
                              // before the last time the
                              // clock was started.
  bool running;               // Is the clock running or
                              // not?

public:

  // Initialize the class variables. At this point no game
  // time has elapsed and the clock is not running.

  gameTime()
  {
    startTime = 0;
    baseTime = 0;
    running = false;
  }

  // Start the clock.

  void start()
  {
    if (!running)
    {
      startTime = SDL_GetTicks();
      running = true;
    }
  }

  // stop the clock

  void stop()
  {
    if (running)
    {
      baseTime = baseTime + (SDL_GetTicks() - startTime);
      running = false;
    }
  }

  // True if the clock is paused.

  bool stopped()
  {
    return !running;
  }

  // Get this clocks current time in milliseconds.

  int time()
  {
    if (running)
    {
      return baseTime + (SDL_GetTicks() - startTime);
    }
    else
    {
      return baseTime;
    }
  }
};

//----------------------------------------------------------

int main(int argc, char **argv)
{

  // Declare all the local variables.

  gameTime gt;
  char *name = argv[0];

  SDL_Surface *screen = NULL;
  SDL_Event event;
  SDL_PixelFormat *pf = NULL;
  Uint32 black;
  Uint32 red;
  Uint32 green;
  Uint32 blue;

  int screenWidth = 640;
  int screenHeight = 480;

  bool done = false;

  float maxv = 100.0, maxa = 10.0;

  simAxis *sa = NULL;

  if (argc > 1)
      maxv = strtod(argv[1], NULL);
  if (argc > 2)
      maxa = strtod(argv[2], NULL);

  // Try to initialize SDL. If it fails, then give up.

  if (-1 == SDL_Init(SDL_INIT_EVERYTHING))
  {
    printf("Can't initialize SDL\n");
    exit(1);
  }

  // Safety first. If the program exits in an unexpected
  // way the atexit() call should ensure that SDL will be
  // shut down properly and the screen returned to a
  // reasonable state.

  atexit(SDL_Quit);

  // Initialize the display. Here I'm asking for a 640x480
  // window with any pixel format and any pixel depth. If
  // you uncomment SDL_FULLSCREEN you should get a 640x480
  // full screen display.

  screen = SDL_SetVideoMode(screenWidth, 
                            screenHeight, 
                            0, 
                            SDL_ANYFORMAT |
                            //SDL_FULLSCREEN |
                            SDL_SWSURFACE
                            );

  if (NULL == screen)
  {
    printf("Can't set video mode\n");
    exit(1);
  }

  // Grab the pixel format for the screen. SDL_MapRGB()
  // needs the pixel format to create pixels that are laid
  // out correctly for the screen.

  pf = screen->format;

  //Create the pixel values used in the program. Black is
  //for clearing the background and the other three are
  //for line colors. Note that in SDL you specify color
  //intensities in the rang 0 to 255 (hex ff). That
  //doesn't mean that you always get 24 or 32 bits of
  //color. If the format doesn't support the full color
  //range, SDL scales it to the range that is correct for
  //the pixel format.

  black = SDL_MapRGB(pf, 0x00, 0x00, 0x00);
  red = SDL_MapRGB(pf, 0xff, 0x00, 0x00);
  green = SDL_MapRGB(pf, 0x00, 0xff, 0x00);
  blue = SDL_MapRGB(pf, 0x00, 0x00, 0xff);

  // Set the window caption and the icon caption for the
  // program. In this case I'm just setting it to whatever
  // the name of the program happens to be.

  SDL_WM_SetCaption(name, name);

  // Create the three animating lines. It is amazing to
  // see the different kinds of behavior you can get from
  // such a simple animation object.

  sa = new simAxis  (screen, maxv, maxa,
                     red, green, blue,
                     gt.time());

  // Start the game clock.

  gt.start();

  // The animation loop.

  while (!done)
  {

    // Loop while reading all pending event.

    while (!done && SDL_PollEvent(&event))
    {
      switch (event.type)
      {

        // Here we are looking for two special keys. If we
        // get an event telling us that the escape key has
        // been pressed the program will quit. If we see
        // the F1 key we either start or stop the
        // animation by starting or stopping the clock.

      case SDL_KEYDOWN:
        switch(event.key.keysym.sym)
        {
        case SDLK_ESCAPE:
          done = true;
          break;

        case SDLK_F1:
          if (gt.stopped())
          {
            gt.start();
          }
          else
          {
            gt.stop();
          }
          break;

        default:
          break;
        }
        break;

        // The SDL_QUIT event is generated when you click
        // on the close button on a window. If we see that
        // event we should exit the program. So, we do.

      case SDL_QUIT:
        done = true;
        break;
      }
    }

    // Erase the old picture by painting the whole buffer
    // black.

    SDL_FillRect(screen, NULL, black);

    // Get the current game time. Note that if the clock
    // is stopped this method will return the same value
    // over and over.

    int t = gt.time();

    // Based on the current time update the location of
    // each line and draw the line into the buffer.

    sa->update(t);

    // Since I'm using a software buffer the call to
    // SDL_Flip() copies the software buffer to the
    // display. That gives you the effect of double
    // buffering without asking for it and without the
    // speed you would get from a hardware double buffered
    // display.

    SDL_Flip(screen);

    // The call to SDL_Delay(10) forces the program to
    // pause for 10 milliseconds and has the effect of
    // limiting the frame rate to less than 100
    // frames/second. It also keeps the program from
    // hogging the CPU which seems to result in smoother
    // animation because the program isn't interrupted by
    // the operating system for long periods.

    SDL_Delay(10);
  }

  // When we get here, just clean up and quit. Yes, the
  // atexit() call makes this redundant. But, it doesn't
  // hurt and I'd rather be safe than sorry.

  SDL_Quit();
}
