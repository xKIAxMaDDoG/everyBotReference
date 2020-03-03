#ifndef __LIGHTING_H__
#define __LIGHTING_H__

/* ************* */
/* CONFIGURATION */
/* ************* */
const int LIGHTING_NUM_PIXELS = 18;

const int LIGHTING_HEADER = 0xFF;


const uint8_t LEFT_SIDE = 1;
const uint8_t RIGHT_SIDE = 2;
const uint8_t BOTH_SIDES = 3;

typedef struct LightingCmdStruct
{
  uint8_t side;
  uint8_t valid;
  uint8_t chase;
  uint8_t start;
  uint8_t width;
  uint8_t left;
  uint8_t right;
} LightingCmd_t;


/* ******************** */
/* LIGHTING DEFINITIONS */
/* ******************** */
const uint8_t LIGHTING_COLOR_ROBONAUTS_GOLD = 0;
const uint8_t LIGHTING_COLOR_RED = 1;
const uint8_t LIGHTING_COLOR_BLUE = 2;
const uint8_t LIGHTING_COLOR_WHITE =  3;
const uint8_t LIGHTING_COLOR_GREEN = 4;
const uint8_t LIGHTING_COLOR_BLACK = 5;

const uint8_t CMD_SET_GAME_DATA = 23;
const uint8_t CMD_SET_AUTON_DATA = 24;
const uint8_t CMD_SET_ALLIANCE_DATA = 25;
const uint8_t CMD_SONAR_ENABLE = 26;
const uint8_t CMD_SET_GAME_MODE_AUTON = 27;
const uint8_t CMD_SET_INTAKE_LIGHTS_ACTIVE = 28;
const uint8_t CMD_SET_LINE = 29;
const uint8_t CMD_SET_LIGHTS = 30;

#endif // __LIGHTING_H__
