/**************************************************************************/
/* 
  Tabular sinbe computation is from AdaFruit (sinewave.pde)
  The rest is mine. 
    @author   Eric
    @license  Who cares?
*/


#include <Wire.h>
#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac1,dac2;


//
#define PI_OVER_4 (3.141516/4.0)

// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution
#define DAC_RESOLUTION    (9)

/* Note: If flash space is tight a quarter sine wave is enough
   to generate full sine and cos waves, but some additional
   calculation will be required at each step after the first
   quarter wave.                                              */

const PROGMEM uint16_t DACLookup_FullSine_9Bit[512] =
{
  2048, 2073, 2098, 2123, 2148, 2174, 2199, 2224,
  2249, 2274, 2299, 2324, 2349, 2373, 2398, 2423,
  2448, 2472, 2497, 2521, 2546, 2570, 2594, 2618,
  2643, 2667, 2690, 2714, 2738, 2762, 2785, 2808,
  2832, 2855, 2878, 2901, 2924, 2946, 2969, 2991,
  3013, 3036, 3057, 3079, 3101, 3122, 3144, 3165,
  3186, 3207, 3227, 3248, 3268, 3288, 3308, 3328,
  3347, 3367, 3386, 3405, 3423, 3442, 3460, 3478,
  3496, 3514, 3531, 3548, 3565, 3582, 3599, 3615,
  3631, 3647, 3663, 3678, 3693, 3708, 3722, 3737,
  3751, 3765, 3778, 3792, 3805, 3817, 3830, 3842,
  3854, 3866, 3877, 3888, 3899, 3910, 3920, 3930,
  3940, 3950, 3959, 3968, 3976, 3985, 3993, 4000,
  4008, 4015, 4022, 4028, 4035, 4041, 4046, 4052,
  4057, 4061, 4066, 4070, 4074, 4077, 4081, 4084,
  4086, 4088, 4090, 4092, 4094, 4095, 4095, 4095,
  4095, 4095, 4095, 4095, 4094, 4092, 4090, 4088,
  4086, 4084, 4081, 4077, 4074, 4070, 4066, 4061,
  4057, 4052, 4046, 4041, 4035, 4028, 4022, 4015,
  4008, 4000, 3993, 3985, 3976, 3968, 3959, 3950,
  3940, 3930, 3920, 3910, 3899, 3888, 3877, 3866,
  3854, 3842, 3830, 3817, 3805, 3792, 3778, 3765,
  3751, 3737, 3722, 3708, 3693, 3678, 3663, 3647,
  3631, 3615, 3599, 3582, 3565, 3548, 3531, 3514,
  3496, 3478, 3460, 3442, 3423, 3405, 3386, 3367,
  3347, 3328, 3308, 3288, 3268, 3248, 3227, 3207,
  3186, 3165, 3144, 3122, 3101, 3079, 3057, 3036,
  3013, 2991, 2969, 2946, 2924, 2901, 2878, 2855,
  2832, 2808, 2785, 2762, 2738, 2714, 2690, 2667,
  2643, 2618, 2594, 2570, 2546, 2521, 2497, 2472,
  2448, 2423, 2398, 2373, 2349, 2324, 2299, 2274,
  2249, 2224, 2199, 2174, 2148, 2123, 2098, 2073,
  2048, 2023, 1998, 1973, 1948, 1922, 1897, 1872,
  1847, 1822, 1797, 1772, 1747, 1723, 1698, 1673,
  1648, 1624, 1599, 1575, 1550, 1526, 1502, 1478,
  1453, 1429, 1406, 1382, 1358, 1334, 1311, 1288,
  1264, 1241, 1218, 1195, 1172, 1150, 1127, 1105,
  1083, 1060, 1039, 1017,  995,  974,  952,  931,
   910,  889,  869,  848,  828,  808,  788,  768,
   749,  729,  710,  691,  673,  654,  636,  618,
   600,  582,  565,  548,  531,  514,  497,  481,
   465,  449,  433,  418,  403,  388,  374,  359,
   345,  331,  318,  304,  291,  279,  266,  254,
   242,  230,  219,  208,  197,  186,  176,  166,
   156,  146,  137,  128,  120,  111,  103,   96,
    88,   81,   74,   68,   61,   55,   50,   44,
    39,   35,   30,   26,   22,   19,   15,   12,
    10,    8,    6,    4,    2,    1,    1,    0,
     0,    0,    1,    1,    2,    4,    6,    8,
    10,   12,   15,   19,   22,   26,   30,   35,
    39,   44,   50,   55,   61,   68,   74,   81,
    88,   96,  103,  111,  120,  128,  137,  146,
   156,  166,  176,  186,  197,  208,  219,  230,
   242,  254,  266,  279,  291,  304,  318,  331,
   345,  359,  374,  388,  403,  418,  433,  449,
   465,  481,  497,  514,  531,  548,  565,  582,
   600,  618,  636,  654,  673,  691,  710,  729,
   749,  768,  788,  808,  828,  848,  869,  889,
   910,  931,  952,  974,  995, 1017, 1039, 1060,
  1083, 1105, 1127, 1150, 1172, 1195, 1218, 1241,
  1264, 1288, 1311, 1334, 1358, 1382, 1406, 1429,
  1453, 1478, 1502, 1526, 1550, 1575, 1599, 1624,
  1648, 1673, 1698, 1723, 1747, 1772, 1797, 1822,
  1847, 1872, 1897, 1922, 1948, 1973, 1998, 2023
};

const PROGMEM uint16_t DACLookup_FullSine_8Bit[256] =
{
  2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398,
  2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
  2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143,
  3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
  3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722,
  3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
  3939, 3958, 3975, 3992, 4007, 4021, 4034, 4045,
  4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
  4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065,
  4056, 4045, 4034, 4021, 4007, 3992, 3975, 3958,
  3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
  3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
  3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226,
  3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
  2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496,
  2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
  2048, 1997, 1947, 1897, 1847, 1797, 1747, 1697,
  1648, 1599, 1550, 1501, 1453, 1405, 1358, 1311,
  1264, 1218, 1172, 1127, 1082, 1038,  995,  952,
   910,  869,  828,  788,  749,  710,  672,  636,
   600,  565,  530,  497,  465,  433,  403,  373,
   345,  318,  291,  266,  242,  219,  197,  176,
   156,  137,  120,  103,   88,   74,   61,   50,
    39,   30,   22,   15,   10,    6,    2,    1,
     0,    1,    2,    6,   10,   15,   22,   30,
    39,   50,   61,   74,   88,  103,  120,  137,
   156,  176,  197,  219,  242,  266,  291,  318,
   345,  373,  403,  433,  465,  497,  530,  565,
   600,  636,  672,  710,  749,  788,  828,  869,
   910,  952,  995, 1038, 1082, 1127, 1172, 1218,
  1264, 1311, 1358, 1405, 1453, 1501, 1550, 1599,
  1648, 1697, 1747, 1797, 1847, 1897, 1947, 1997
};


/* 
  Full range is [0,4095] on the DAC (12 bits)
  Since we use 8-bit values, we multiply every coordinate by 16. 
*/
void plot(int16_t x, int16_t y)
{
  dac1.setVoltage(x, false);
  dac2.setVoltage(y, false);
}

#define len_seg_square 50

void vector_2(float x1, float y1, float x2, float y2)
{
  int n = sqrt(((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)) / (len_seg_square));
  
  float dx= ((float)x2-(float)x1) / n;
  float dy= ((float)y2-(float)y1) / n;
  
  for (int i=0; i< n; i++)
    plot( (float) i*dx+x1, (float) i*dy+y1);

}
/*  Quadrants
    2  |  1
-------|-----
    3  |  4

*/
void vector(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
    // optimized implementation of Bresenham's Algorithm
    // also known as a DDA - digital differential analyzer
    int acc;


    // for speed, there are 8 DDA's, one for each octant
    if (y1 < y2) { // quadrant 1 or 2
        byte dy = y2 - y1;
        if (x1 < x2) { // quadrant 1
            byte dx = x2 - x1;
            if (dx > dy) { // < 45
                acc = (dx >> 1);
                for (; x1 < x2; x1++) {
                    plot(x1, y1);
                    acc -= dy;
                    if (acc < 0) {
                        y1++;
                        acc += dx;
                    }
                }
            }
            else {   // > 45
                acc = dy >> 1;
                for (; y1 < y2; y1++) {
                    plot(x1, y1);
                    acc -= dx;
                    if (acc < 0) {
                        x1++;
                        acc += dy;
                    }
                }
            }
        }
        else {  // quadrant 2
            byte dx = x1 - x2;
            if (dx > dy) { // < 45
                acc = dx >> 1;
                for (; x1 > x2; x1--) {
                    plot(x1, y1);
                    acc -= dy;
                    if (acc < 0) {
                        y1++;
                        acc += dx;
                    }
                }
            }
            else {  // > 45
                acc = dy >> 1;
                for (; y1 < y2; y1++) {
                    plot(x1, y1);
                    acc -= dx;
                    if (acc < 0) {
                        x1--;
                        acc += dy;
                    }
                }
            }        
        }
    }
    else { // quadrant 3 or 4
        byte dy = y1 - y2;
        if (x1 < x2) { // quadrant 4
            byte dx = x2 - x1;
            if (dx > dy) {  // < 45
                acc = dx >> 1;
                for (; x1 < x2; x1++) {
                    plot(x1, y1);
                    acc -= dy;
                    if (acc < 0) {
                        y1--;
                        acc += dx;
                    }
                }
            
            }
            else {  // > 45
                acc = dy >> 1;
                for (; y1 > y2; y1--) {
                    plot(x1, y1);
                    acc -= dx;
                    if (acc < 0) {
                        x1++;
                        acc += dy;
                    }
                }

            }
        }
        else {  // quadrant 3
            byte dx = x1 - x2;
            if (dx > dy) { // < 45
                acc = dx >> 1;
                for (; x1 > x2; x1--) {
                    plot(x1, y1);
                    acc -= dy;
                    if (acc < 0) {
                        y1--;
                        acc += dx;
                    }
                }

            }
            else {  // > 45
                acc = dy >> 1;
                for (; y1 > y2; y1--) {
                    plot(x1, y1);
                    acc -= dx;
                    if (acc < 0) {
                        x1--;
                        acc += dy;
                    }
                }
            }
        }
    
    }
}

/*
 * Rotate a set of points (ix[],iy[]) around (0,0) and place the result in (ox[],oy[]). 
 */
void rotate (float ix[], float iy[], float ox[], float oy[], float angle, int n) {
    
    angle *= 3.1416 / 180.0;
    
    for ( int i = 0; i< n; i++)
    {
      ox[i] = (float) (ix[i]) * cos(angle) + (float)(iy[i]) * sin(angle);
      oy[i] = - (float)(ix[i]) * sin(angle) + (float) (iy[i]) * cos (angle);
    }
    
}


#define LEFTBUT 4
#define FIREBUT 5
#define RIGHTBUT 6




/*
 * 
 */
void setup(void) {
  Serial.begin(9600);

  // Setup input pins
  pinMode(LEFTBUT, INPUT_PULLUP); 
  pinMode(FIREBUT, INPUT_PULLUP);
  pinMode(RIGHTBUT, INPUT_PULLUP);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac1.begin(0x60);
  dac2.begin(0x61);

}

// Polygon representing the shape of the ship 
float orgshipx[]={50,-50,-50, 50,0};
float orgshipy[]={0,-50, 50, 0, 0};

// Polygon representing the ship at its actual location
float shipx[5];
float shipy[5];


typedef enum {e_bomb_off, e_bomb_init,e_bomb_move } 
  bomb_state_e;

bomb_state_e bomb_state= e_bomb_off;

// Polygon representing the shape of the ship 
float orgbombx[]={10,10,-10, -10,10};
float orgbomby[]={10,-10,-10, 10, 10};

// Polygon representing the ship at its actual location
float bombx[5];
float bomby[5];
  

// Screen is [0,255]x[0,255]
void loop(void) {
    static uint16_t cpt=0;
    static uint8_t firecpt=0, leftcpt=0, rightcpt=0;
    static float ox=2048.0, oy=2048.0, bx, by;
    static float vx=0, vy=0, bvx, bvy;
    static float  speed=0.0;
    static float angle = 0.0;
    static float anginc, speedinc;
    float nx,ny;
    boolean angle_changed = true, speed_changed = true;

  if (digitalRead(FIREBUT) == LOW) {
    firecpt++;
    if (firecpt == 10) { 
      if ( speedinc < 20.0) speedinc += 0.1;
      firecpt = 0;
    }
    speed += speedinc;
    speed_changed = true;
  }
  else
  {
    speedinc = 0.5;
    if (firecpt != 0) {
      Serial.println("BOMB!");
        bomb_state = e_bomb_init;
    }
    firecpt=0;
  }


 
 if (digitalRead(LEFTBUT) == LOW) {
    leftcpt++;
    if (leftcpt == 10) { 
      if ( anginc < 10.0) anginc += 1.0;
      leftcpt=0;
    }
      angle -= anginc;
      angle_changed = true;
  }
  else {
    leftcpt=0;
   
    if (digitalRead(RIGHTBUT) == LOW) {
      rightcpt++;
      if (rightcpt == 10) { 
        if ( anginc < 10) anginc += 1;
        rightcpt=0;
      }
        angle += anginc;
        angle_changed = true;

    }
    else {
      rightcpt=0;
      anginc = 1;
    }
  }



   if ( angle_changed) {
    rotate (orgshipx, orgshipy, shipx, shipy, angle, 5);
   }
   
  // Draw ship
  for (int i=0; i<4; i++)
    vector_2(shipx[i]+ox, shipy[i]+oy, shipx[i+1]+ox, shipy[i+1]+oy);

  if ( angle_changed || speed_changed ) {
    vx = speed* cos(angle* 3.1416 / 180.0);
    vy = -speed* sin(angle* 3.1416 / 180.0);
  }
    nx = ox+vx;
    ny = oy+vy;

    if ((nx<200.0) || (nx>3900.0) || (ny<200.0) || (ny>3900.0)) speed= 0.0;
      
   else {
    ox = nx;
    oy = ny;
    }


  
  switch (bomb_state) {
    case e_bomb_off:
      break;
    case e_bomb_init:
      rotate (orgbombx, orgbomby, bombx, bomby, angle, 4);
      bx= shipx[0]+ox;
      by= shipy[0]+oy;
      bvx = (speed+10)* cos(angle* 3.1416 / 180.0);
      bvy = -(speed+10)* sin(angle* 3.1416 / 180.0);
      bomb_state = e_bomb_move;
      break;
    case e_bomb_move:
      bx += bvx;
      by += bvy;
     if ((bx<200.0) || (bx>3900.0) || (by<200.0) || (by>3900.0))
        bomb_state = e_bomb_off;
      else
     if ( (bx-ox)*(bx-ox)+(by-oy)*(by-oy) > 1000000 )
        bomb_state = e_bomb_off;
     else
       // Draw bomb
        for (int i=0; i<4; i++)
          vector_2(bombx[i]+bx, bomby[i]+by, bombx[i+1]+bx, bomby[i+1]+by);
      break;      
    default:
      break;
  }

    
    cpt++;
    if ((cpt & 0x1F) == 0) {
      speed *=0.75;
        speed_changed = true;
      }
  
 }


 /*
// X in [0,511]
x1= x1+32;
x2= (x1+512);


v1 = pgm_read_word(&(DACLookup_FullSine_9Bit[(x1>>2) & 511]));
v2 = pgm_read_word(&(DACLookup_FullSine_9Bit[(x2>>2) & 511]));

v1 = (((float)(v1)-2048)/4096.0)*s+2048.0+ox ;
v2 = (((float)(v2)-2048.0)/4096.0)*s+2048+oy ;

    dac1.setVoltage(v1, false);
    dac2.setVoltage(v2, false);

  */    
