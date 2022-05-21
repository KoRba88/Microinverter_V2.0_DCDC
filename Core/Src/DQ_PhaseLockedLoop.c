/*
 * DQ_PhaseLockedLoop.c
 *
 *  Created on: Aug 6, 2020
 *      Author: user01
 */
#include "main.h"
//#include "Solar_define.h"
#include "Solar_Mul_Div.h"
#include "400WControl.h"
#include "DAC_Debug.h"
#include "DQ_PhaseLockedLoop.h"

//static u16 ii = 0;
//uint16_t buffer_1[2048] = {0};
//uint16_t buffer_2[2048] = {0};
//u16 iii;

s16 SimpleMovingAvarage_DC_BUS(s16 Volt_Output_nofiltered_DC_BUS);
#define samples_DC_BUS 16
#define Delay90 (-((SAMPLINGFREQ/50/4)-256))

#define Delay10 (-((SAMPLINGFREQ/50/36)-256))

SystStatus_t Freq_Control= FREQ_OUT_OF_RANGE;
SystStatus_t GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
extern s16 Grid_Voltage_max;
extern s16 Grid_Voltage_min;
extern SystStatus_t Diagnostic_Control;

static u16 tti =0;
int32_t theta_total[4096] ={0};
static u32 sti =0;

s16 temp_Index_Cos=0;
s16 temp_Index_Sin=0;

s16 qV_Direct_filtered=0;
s16 qV_Quadrature_filtered=0;

#define new_mul_q15_q15_q31(Op1, Op2) (((s32)((s32)Op1*(s32)Op2))>=0x40000000 ? 0x7fffffff :(((s32)((s32)Op1*(s32)Op2))<<1))


  //tabella USATA x IL VECTOR
// 0x0000,0x0324,0x0647,0x096A,0x0C8B,0x0FAA,0x12C7,0x15E1,0x18F7,0x1C0A,0x1F18,0x2222,0x2526,0x2824,0x2B1D,0x2E0F,0x30F9,0x33DC,
//0x36B7,0x398A,0x3C54,0x3F14,0x41CB,0x4478,0x471A,0x49B1,0x4C3C,0x4EBC,0x5130,0x5398,0x55F2,0x583F,0x5A7F,0x5CB0,0x5ED4,0x60E8,
//0x62EE,0x64E5,0x66CC,0x68A3,0x6A6A,0x6C21,0x6DC6,0x6F5B,0x70DF,0x7252,0x73B3,0x7502,0x763F,0x7769,0x7882,0x7987,0x7A7A,0x7B5A,
//0x7C28,0x7CE2,0x7D88,0x7E1C,0x7E9C,0x7F08,0x7F61,0x7FA6,0x7FD8,0x7FF5,0x7FFF,0x7FD8,0x7FA7,0x7F63,0x7F0A,0x7E9E,0x7E1F,0x7D8C,
//0x7CE6,0x7C2C,0x7B5F,0x7A80,0x798D,0x7888,0x7770,0x7646,0x7509,0x73BA,0x725A,0x70E8,0x6F64,0x6DD0,0x6C2A,0x6A74,0x68AD,0x66D7,
//0x64F0,0x62FA,0x60F4,0x5EE0,0x5CBD,0x5A8C,0x584C,0x5600,0x53A5,0x513E,0x4ECB,0x4C4B,0x49C0,0x4729,0x4487,0x41DB,0x3F24,0x3C64,
//0x399A,0x36C8,0x33ED,0x310A,0x2E20,0x2B2E,0x2836,0x2537,0x2233,0x1F2A,0x1C1C,0x1909,0x15F3,0x12D9,0x0FBC,0x0C9D,0x097C,0x0659,
//0x0336,0x0012,0xFCEF,0xF9CB,0xF6A9,0xF387,0xF068,0xED4B,0xEA31,0xE71B,0xE408,0xE0FA,0xDDF0,0xDAEC,0xD7ED,0xD4F4,0xD203,0xCF18,
//0xCC35,0xC959,0xC686,0xC3BC,0xC0FC,0xBE45,0xBB98,0xB8F6,0xB65E,0xB3D2,0xB152,0xAEDE,0xAC76,0xAA1C,0xA7CE,0xA58E,0xA35C,0xA139,
//0x9F24,0x9D1D,0x9B26,0x993F,0x9768,0x95A0,0x93E9,0x9243,0x90AE,0x8F29,0x8DB6,0x8C55,0x8B06,0x89C8,0x889D,0x8785,0x867E,0x858B,
//0x84AA,0x83DD,0x8322,0x827B,0x81E7,0x8167,0x80FA,0x80A1,0x805B,0x8029,0x800B,0x8001,0x800A,0x8027,0x8057,0x809C,0x80F3,0x815F,
//0x81DE,0x8271,0x8316,0x83D0,0x849C,0x857B,0x866D,0x8772,0x888A,0x89B3,0x8AF0,0x8C3E,0x8D9E,0x8F0F,0x9092,0x9227,0x93CC,0x9582,
//0x9748,0x991E,0x9B05,0x9CFB,0x9F00,0xA114,0xA336,0xA567,0xA7A6,0xA9F3,0xAC4D,0xAEB4,0xB127,0xB3A6,0xB631,0xB8C8,0xBB6A,0xBE16,
//0xC0CC,0xC38C,0xC655,0xC928,0xCC02,0xCEE5,0xD1CF,0xD4C1,0xD7B9,0xDAB7,0xDDBB,0xE0C4,0xE3D3,0xE6E5,0xE9FB,0xED15,0xF032,0xF351,
//0xF672,0xF994,0xFCB8,0xFFDC,
//};
//




//0xFCEF,0xF9CB,0xF6A9,0xF387,0xF068,0xED4B,0xEA31,0xE71B,0xE408,0xE0FA,0xDDF0,0xDAEC,0xD7ED,0xD4F4,0xD203,0xCF18,
//0xCC35,0xC959,0xC686,0xC3BC,0xC0FC,0xBE45,0xBB98,0xB8F6,0xB65E,0xB3D2,0xB152,0xAEDE,0xAC76,0xAA1C,0xA7CE,0xA58E,0xA35C,0xA139,
//0x9F24,0x9D1D,0x9B26,0x993F,0x9768,0x95A0,0x93E9,0x9243,0x90AE,0x8F29,0x8DB6,0x8C55,0x8B06,0x89C8,0x889D,0x8785,0x867E,0x858B,
//0x84AA,0x83DD,0x8322,0x827B,0x81E7,0x8167,0x80FA,0x80A1,0x805B,0x8029,0x800B,0x8001,0x800A,0x8027,0x8057,0x809C,0x80F3,0x815F,
//0x81DE,0x8271,0x8316,0x83D0,0x849C,0x857B,0x866D,0x8772,0x888A,0x89B3,0x8AF0,0x8C3E,0x8D9E,0x8F0F,0x9092,0x9227,0x93CC,0x9582,
//0x9748,0x991E,0x9B05,0x9CFB,0x9F00,0xA114,0xA336,0xA567,0xA7A6,0xA9F3,0xAC4D,0xAEB4,0xB127,0xB3A6,0xB631,0xB8C8,0xBB6A,0xBE16,
//0xC0CC,0xC38C,0xC655,0xC928,0xCC02,0xCEE5,0xD1CF,0xD4C1,0xD7B9,0xDAB7,0xDDBB,0xE0C4,0xE3D3,0xE6E5,0xE9FB,0xED15,0xF032,0xF351,
//0xF672,0xF994,0xFCB8,0xFFDC,0x0,0x0324,0x0647,0x096A,0x0C8B,0x0FAA,0x12C7,0x15E1,0x18F7,0x1C0A,0x1F18,0x2222,0x2526,0x2824,
//0x2B1D,0x2E0F,0x30F9,0x33DC,0x36B7,0x398A,0x3C54,0x3F14,0x41CB,0x4478,0x471A,0x49B1,0x4C3C,0x4EBC,0x5130,0x5398,0x55F2,0x583F,
//0x5A7F,0x5CB0,0x5ED4,0x60E8,0x62EE,0x64E5,0x66CC,0x68A3,0x6A6A,0x6C21,0x6DC6,0x6F5B,0x70DF,0x7252,0x73B3,0x7502,0x763F,0x7769,
//0x7882,0x7987,0x7A7A,0x7B5A,0x7C28,0x7CE2,0x7D88,0x7E1C,0x7E9C,0x7F08,0x7F61,0x7FA6,0x7FD8,0x7FF5,0x7FFF,      0x7FD8,0x7FA7,
//0x7F63,0x7F0A,0x7E9E,0x7E1F,0x7D8C,0x7CE6,0x7C2C,0x7B5F,0x7A80,0x798D,0x7888,0x7770,0x7646,0x7509,0x73BA,0x725A,0x70E8,0x6F64,
//0x6DD0,0x6C2A,0x6A74,0x68AD,0x66D7,0x64F0,0x62FA,0x60F4,0x5EE0,0x5CBD,0x5A8C,0x584C,0x5600,0x53A5,0x513E,0x4ECB,0x4C4B,0x49C0,
//0x4729,0x4487,0x41DB,0x3F24,0x3C64,0x399A,0x36C8,0x33ED,0x310A,0x2E20,0x2B2E,0x2836,0x2537,0x2233,0x1F2A,0x1C1C,0x1909,0x15F3,
//0x12D9,0x0FBC,0x0C9D,0x097C,0x0659,0x0336,0x0012

//*************************** TAB DELETED BY G.S.
// const s16 Sin_Cos_Table[STEPS] = {
//0xFCEF,	0xF9CB,	0xF6A9,	0xF387,	0xF068,	0xED4B,	0xEA31,	0xE71B,	0xE408,	0xE0FA,	0xDDF0,	0xDAEC,	0xD7ED,	0xD4F4,	0xD203,	0xCF18,
//0xCC35,	0xC959,	0xC686,	0xC3BC,	0xC0FC,	0xBE45,	0xBB98,	0xB8F6,	0xB65E,	0xB3D2,	0xB152,	0xAEDE,	0xAC76,	0xAA1C,	0xA7CE,	0xA58E,
//0xA35C,	0xA139, 0x9F24,	0x9D1D,	0x9B26,	0x993F,	0x9768,	0x95A0,	0x93E9,	0x9243,	0x90AE,	0x8F29,	0x8DB6,	0x8C55,	0x8B06,
//0x89C8,	0x889D,	0x8785,	0x867E,	0x858B, 0x84AA,	0x83DD,	0x8322,	0x827B,	0x81E7,	0x8167,	0x80FA,	0x80A1,	0x805B,	0x8029,
//0x800B,	0x8001,	0x800A,	0x8027,	0x8057,	0x809C,	0x80F3,	0x815F,	0x81DE,	0x8271,	0x8316,	0x83D0,	0x849C,	0x857B,	0x866D,	0x8772,
//0x888A,	0x89B3,	0x8AF0,	0x8C3E,	0x8D9E,	0x8F0F,	0x9092,	0x9227,	0x93CC,	0x9582,	0x9748,	0x991E,	0x9B05,	0x9CFB,	0x9F00,	0xA114,
//0xA336,	0xA567,	0xA7A6,	0xA9F3,	0xAC4D,	0xAEB4,	0xB127,	0xB3A6,	0xB631,	0xB8C8,	0xBB6A,	0xBE16, 0xC0CC,	0xC38C,	0xC655,
//0xC928,	0xCC02,	0xCEE5,	0xD1CF,	0xD4C1,	0xD7B9,	0xDAB7,	0xDDBB,	0xE0C4,	0xE3D3,	0xE6E5,	0xE9FB,	0xED15,	0xF032,	0xF351,	0xF672,
//0xF994,	0xFCB8,	0xFFDC,	0x0,	0x324,	0x647,	0x096A,	0x0C8B,	0x0FAA,	0x12C7,	0x15E1,	0x18F7,	0x1C0A,	0x1F18,	0x2222,	0x2526,
//0x2824,	0x2B1D,	0x2E0F,	0x30F9,	0x33DC,	0x36B7,	0x398A,	0x3C54,	0x3F14,	0x41CB,	0x4478,	0x471A,	0x49B1,	0x4C3C,	0x4EBC,	0x5130,
//0x5398,	0x55F2,	0x583F,	0x5A7F,	0x5CB0,	0x5ED4,	0x60E8,	0x62EE,	0x64E5,	0x66CC,	0x68A3,	0x6A6A,	0x6C21,	0x6DC6,	0x6F5B,	0x70DF,
//0x7252,	0x73B3,	0x7502,	0x763F,	0x7769,	0x7882,	0x7987,	0x7A7A,	0x7B5A,	0x7C28,	0x7CE2,	0x7D88,	0x7E1C,	0x7E9C,	0x7F08,	0x7F61,
//0x7FA6,	0x7FD8,	0x7FF5,	0x7FFF, 0x7FD8,	0x7FA7,	0x7F63,	0x7F0A,	0x7E9E,	0x7E1F,	0x7D8C,	0x7CE6,	0x7C2C,	0x7B5F,	0x7A80,
//0x798D,	0x7888,	0x7770,	0x7646,	0x7509,	0x73BA,	0x725A,	0x70E8,	0x6F64,	0x6DD0,	0x6C2A,	0x6A74,	0x68AD,	0x66D7,	0x64F0,	0x62FA,
//0x60F4,	0x5EE0,	0x5CBD,	0x5A8C,	0x584C,	0x5600,	0x53A5,	0x513E,	0x4ECB,	0x4C4B,	0x49C0,	0x4729,	0x4487,	0x41DB,	0x3F24,	0x3C64,
//0x399A,	0x36C8,	0x33ED,	0x310A,	0x2E20,	0x2B2E,	0x2836,	0x2537,	0x2233,	0x1F2A,	0x1C1C,	0x1909,	0x15F3,	0x12D9,	0x0FBC,	0x0C9D,
//0x097C,	0x659,	0x336,	0x12,
//
//};

//********************** END  TAB DELETED BY G.S.

//********************************* 351 elements
//const s16 Sin_Cos_Table[STEPS] = {
//0x10034,0xFDE8,	0xFB9C,	0xF951,	0xF706,	0xF4BC,	0xF273,	0xF02B,	0xEDE4,	0xEB9F,	0xE95B,	0xE719,	0xE4DA,	0xE29C,	0xE061,	0xDE29,
//0xDBF3,	0xD9C1,	0xD791,	0xD565,	0xD33C,	0xD118,	0xCEF6,	0xCCD9,	0xCAC1,	0xC8AC,	0xC69C,	0xC491,	0xC28B,	0xC08A,	0xBE8E,	0xBC97,
//0xBAA6,	0xB8BB,	0xB6D6,	0xB4F6,	0xB31D,	0xB14A,	0xAF7E,	0xADB8,	0xABF9,	0xAA41,	0xA890,	0xA6E6,	0xA544,	0xA3A9,	0xA216,	0xA08A,
//0x9F06,	0x9D8B,	0x9C17,	0x9AAC,	0x9949,	0x97EE,	0x969C,	0x9553,	0x9412,	0x92DB,	0x91AC,	0x9087,	0x8F6A,	0x8E57,	0x8D4E,	0x8C4D,
//0x8B57,	0x8A6A,	0x8986,	0x88AD,	0x87DD,	0x8717,	0x865B,	0x85A9,	0x8501,	0x8463,	0x83D0,	0x8347,	0x82C8,	0x8253,	0x81E9,	0x8189,
//0x8133,	0x80E8,	0x80A8,	0x8072,	0x8046,	0x8025,	0x800F,	0x8003,	0x8001,	0x800A,	0x801E,	0x803C,	0x8065,	0x8098,	0x80D6,	0x811E,
//0x8171,	0x81CE,	0x8236,	0x82A8,	0x8324,	0x83AA,	0x843B,	0x84D6,	0x857B,	0x862B,	0x86E4,	0x87A7,	0x8874,	0x894B,	0x8A2C,	0x8B17,
//0x8C0B,	0x8D08,	0x8E10,	0x8F20,	0x903A,	0x915D,	0x9289,	0x93BF,	0x94FD,	0x9644,	0x9793,	0x98EC,	0x9A4C,	0x9BB6,	0x9D27,	0x9EA1,
//0xA022,	0xA1AC,	0xA33D,	0xA4D6,	0xA676,	0xA81E,	0xA9CD,	0xAB83,	0xAD40,	0xAF04,	0xB0CF,	0xB2A0,	0xB478,	0xB655,	0xB839,	0xBA23,
//0xBC12,	0xBE07,	0xC002,	0xC202,	0xC407,	0xC611,	0xC81F,	0xCA32,	0xCC4A,	0xCE66,	0xD086,	0xD2AA,	0xD4D2,	0xD6FD,	0xD92B,	0xDB5D,
//0xDD92,	0xDFCA,	0xE204,	0xE441,	0xE680,	0xE8C1,	0xEB04,	0xED49,	0xEF8F,	0xF1D7,	0xF420,	0xF66A,	0xF8B4,	0xFB00,	0xFD4B,	0x0,
//0x24B,	0x497,	0x6E2,	0x92D,	0xB77,	0xDC0,	0x1008,	0x124F,	0x1494,	0x16D7,	0x1919,	0x1B58,	0x1D96,	0x1FD0,	0x2208,	0x243E,
//0x2670,	0x289F,	0x2ACB,	0x2CF3,	0x2F18,	0x3139,	0x3355,	0x356E,	0x3782,	0x3991,	0x3B9C,	0x3DA2,	0x3FA3,	0x419E,	0x4394,	0x4585,
//0x476F,	0x4954,	0x4B33,	0x4D0C,	0x4EDE,	0x50AA,	0x526F,	0x542D,	0x55E5,	0x5795,	0x593E,	0x5AE0,	0x5C7A,	0x5E0D,	0x5F98,	0x611B,
//0x6296,	0x6408,	0x6573,	0x66D5,	0x682F,	0x6980,	0x6AC9,	0x6C09,	0x6D3F,	0x6E6D,	0x6F92,	0x70AD,	0x71C0,	0x72C9,	0x73C8,	0x74BE,
//0x75AA,	0x768D,	0x7765,	0x7834,	0x78F9,	0x79B4,	0x7A65,	0x7B0C,	0x7BA9,	0x7C3C,	0x7CC4,	0x7D42,	0x7DB6,	0x7E1F,	0x7E7E,	0x7ED3,
//0x7F1D,	0x7F5C,	0x7F92,	0x7FBC,	0x7FDC,	0x7FF2,	0x7FFD,	0x7FFD,	0x7FF3,	0x7FDF,	0x7FC0,	0x7F96,	0x7F62,	0x7F23,	0x7EDA,	0x7E86,
//0x7E28,	0x7DC0,	0x7D4D,	0x7CD0,	0x7C48,	0x7BB6,	0x7B1B,	0x7A75,	0x79C4,	0x790A,	0x7846,	0x7778,	0x76A0,	0x75BF,	0x74D3,	0x73DE,
//0x72E0,	0x71D8,	0x70C6,	0x6FAC,	0x6E88,	0x6D5B,	0x6C25,	0x6AE6,	0x699E,	0x684D,	0x66F4,	0x6593,	0x6429,	0x62B7,	0x613D,	0x5FBA,
//0x5E30,	0x5C9E,	0x5B05,	0x5963,	0x57BB,	0x560B,	0x5454,	0x5297,	0x50D2,	0x4F07,	0x4D35,	0x4B5D,	0x497F,	0x479B,	0x45B0,	0x43C0,
//0x41CB,	0x3FD0,	0x3DD0,	0x3BCA,	0x39C0,	0x37B1,	0x359D,	0x3385,	0x3169,	0x2F49,	0x2D24,	0x2AFC,	0x28D1,	0x26A2,	0x2470,	0x223B,
//0x2003,	0x1DC8,	0x1B8B,	0x194C,	0x170B,	0x14C7,	0x1282,	0x103C,	0xDF4,	0xBAB,	0x961,	0x717,	0x4CB,	0x280,
//};
//*****************************************************************************



//*********************** 512 elements ******************************************
const s16 Sin_Cos_Table[STEPS] = {
//tabella USATA x IL VECTOR
0x0000, 0xFE6E, 0xFCDC, 0xFB4A, 0xF9B9, 0xF827, 0xF696, 0xF505, 0xF375, 0xF1E5, 0xF055, 0xEEC7, 0xED38, 0xEBAB, 0xEA1E, 0xE893,
0xE708, 0xE57E, 0xE3F5, 0xE26D, 0xE0E7, 0xDF61, 0xDDDD, 0xDC5A, 0xDAD8, 0xD958, 0xD7DA, 0xD65D, 0xD4E1, 0xD368, 0xD1EF, 0xD079,
0xCF05, 0xCD92, 0xCC22, 0xCAB3, 0xC946, 0xC7DC, 0xC674, 0xC50E, 0xC3AA, 0xC248, 0xC0E9, 0xBF8D, 0xBE32, 0xBCDB, 0xBB86, 0xBA33,
0xB8E4, 0xB797, 0xB64C, 0xB505, 0xB3C1, 0xB27F, 0xB141, 0xB005, 0xAECD, 0xAD97, 0xAC65, 0xAB36, 0xAA0B, 0xA8E3, 0xA7BE, 0xA69C,
0xA57E, 0xA463, 0xA34C, 0xA239, 0xA129, 0xA01D, 0x9F14, 0x9E0F, 0x9D0E, 0x9C11, 0x9B18, 0x9A23, 0x9931, 0x9843, 0x975A, 0x9674,
0x9593, 0x94B6, 0x93DC, 0x9307, 0x9236, 0x916A, 0x90A1, 0x8FDD, 0x8F1E, 0x8E62, 0x8DAB, 0x8CF9, 0x8C4B, 0x8BA1, 0x8AFC, 0x8A5B,
0x89BF, 0x8927, 0x8894, 0x8806, 0x877C, 0x86F7, 0x8676, 0x85FB, 0x8583, 0x8511, 0x84A3, 0x843B, 0x83D7, 0x8377, 0x831D, 0x82C7,
0x8276, 0x822A, 0x81E3, 0x81A1, 0x8163, 0x812B, 0x80F7, 0x80C8, 0x809E, 0x8079, 0x8059, 0x803E, 0x8028, 0x8017, 0x800A, 0x8003,
0x8000, 0x8003, 0x800A, 0x8017, 0x8028, 0x803E, 0x8059, 0x8079, 0x809E, 0x80C8, 0x80F7, 0x812B, 0x8163, 0x81A1, 0x81E3, 0x822A,
0x8276, 0x82C7, 0x831D, 0x8377, 0x83D7, 0x843B, 0x84A3, 0x8511, 0x8583, 0x85FB, 0x8676, 0x86F7, 0x877C, 0x8806, 0x8894, 0x8927,
0x89BF, 0x8A5B, 0x8AFC, 0x8BA1, 0x8C4B, 0x8CF9, 0x8DAB, 0x8E62, 0x8F1E, 0x8FDD, 0x90A1, 0x916A, 0x9236, 0x9307, 0x93DC, 0x94B6,
0x9593, 0x9674, 0x975A, 0x9843, 0x9931, 0x9A23, 0x9B18, 0x9C11, 0x9D0E, 0x9E0F, 0x9F14, 0xA01D, 0xA129, 0xA239, 0xA34C, 0xA463,
0xA57E, 0xA69C, 0xA7BE, 0xA8E3, 0xAA0B, 0xAB36, 0xAC65, 0xAD97, 0xAECD, 0xB005, 0xB141, 0xB27F, 0xB3C1, 0xB505, 0xB64C, 0xB797,
0xB8E4, 0xBA33, 0xBB86, 0xBCDB, 0xBE32, 0xBF8D, 0xC0E9, 0xC248, 0xC3AA, 0xC50E, 0xC674, 0xC7DC, 0xC946, 0xCAB3, 0xCC22, 0xCD92,
0xCF05, 0xD079, 0xD1EF, 0xD368, 0xD4E1, 0xD65D, 0xD7DA, 0xD958, 0xDAD8, 0xDC5A, 0xDDDD, 0xDF61, 0xE0E7, 0xE26D, 0xE3F5, 0xE57E,
0xE708, 0xE893, 0xEA1E, 0xEBAB, 0xED38, 0xEEC7, 0xF055, 0xF1E5, 0xF375, 0xF505, 0xF696, 0xF827, 0xF9B9, 0xFB4A, 0xFCDC, 0xFE6E,
0x0000, 0x0192, 0x0324, 0x04B6, 0x0647, 0x07D9, 0x096A, 0x0AFB, 0x0C8B, 0x0E1B, 0x0FAB, 0x1139, 0x12C8, 0x1455, 0x15E2, 0x176D,
0x18F8, 0x1A82, 0x1C0B, 0x1D93, 0x1F19, 0x209F, 0x2223, 0x23A6, 0x2528, 0x26A8, 0x2826, 0x29A3, 0x2B1F, 0x2C98, 0x2E11, 0x2F87,
0x30FB, 0x326E, 0x33DE, 0x354D, 0x36BA, 0x3824, 0x398C, 0x3AF2, 0x3C56, 0x3DB8, 0x3F17, 0x4073, 0x41CE, 0x4325, 0x447A, 0x45CD,
0x471C, 0x4869, 0x49B4, 0x4AFB, 0x4C3F, 0x4D81, 0x4EBF, 0x4FFB, 0x5133, 0x5269, 0x539B, 0x54CA, 0x55F5, 0x571D, 0x5842, 0x5964,
0x5A82, 0x5B9D, 0x5CB4, 0x5DC7, 0x5ED7, 0x5FE3, 0x60EC, 0x61F1, 0x62F2, 0x63EF, 0x64E8, 0x65DD, 0x66CF, 0x67BD, 0x68A6, 0x698C,
0x6A6D, 0x6B4A, 0x6C24, 0x6CF9, 0x6DCA, 0x6E96, 0x6F5F, 0x7023, 0x70E2, 0x719E, 0x7255, 0x7307, 0x73B5, 0x745F, 0x7504, 0x75A5,
0x7641, 0x76D9, 0x776C, 0x77FA, 0x7884, 0x7909, 0x798A, 0x7A05, 0x7A7D, 0x7AEF, 0x7B5D, 0x7BC5, 0x7C29, 0x7C89, 0x7CE3, 0x7D39,
0x7D8A, 0x7DD6, 0x7E1D, 0x7E5F, 0x7E9D, 0x7ED5, 0x7F09, 0x7F38, 0x7F62, 0x7F87, 0x7FA7, 0x7FC2, 0x7FD8, 0x7FE9, 0x7FF6, 0x7FFD,
0x7FFF, 0x7FFD, 0x7FF6, 0x7FE9, 0x7FD8, 0x7FC2, 0x7FA7, 0x7F87, 0x7F62, 0x7F38, 0x7F09, 0x7ED5, 0x7E9D, 0x7E5F, 0x7E1D, 0x7DD6,
0x7D8A, 0x7D39, 0x7CE3, 0x7C89, 0x7C29, 0x7BC5, 0x7B5D, 0x7AEF, 0x7A7D, 0x7A05, 0x798A, 0x7909, 0x7884, 0x77FA, 0x776C, 0x76D9,
0x7641, 0x75A5, 0x7504, 0x745F, 0x73B5, 0x7307, 0x7255, 0x719E, 0x70E2, 0x7023, 0x6F5F, 0x6E96, 0x6DCA, 0x6CF9, 0x6C24, 0x6B4A,
0x6A6D, 0x698C, 0x68A6, 0x67BD, 0x66CF, 0x65DD, 0x64E8, 0x63EF, 0x62F2, 0x61F1, 0x60EC, 0x5FE3, 0x5ED7, 0x5DC7, 0x5CB4, 0x5B9D,
0x5A82, 0x5964, 0x5842, 0x571D, 0x55F5, 0x54CA, 0x539B, 0x5269, 0x5133, 0x4FFB, 0x4EBF, 0x4D81, 0x4C3F, 0x4AFB, 0x49B4, 0x4869,
0x471C, 0x45CD, 0x447A, 0x4325, 0x41CE, 0x4073, 0x3F17, 0x3DB8, 0x3C56, 0x3AF2, 0x398C, 0x3824, 0x36BA, 0x354D, 0x33DE, 0x326E,
0x30FB, 0x2F87, 0x2E11, 0x2C98, 0x2B1F, 0x29A3, 0x2826, 0x26A8, 0x2528, 0x23A6, 0x2223, 0x209F, 0x1F19, 0x1D93, 0x1C0B, 0x1A82,
0x18F8, 0x176D, 0x15E2, 0x1455, 0x12C8, 0x1139, 0x0FAB, 0x0E1B, 0x0C8B, 0x0AFB, 0x096A, 0x07D9, 0x0647, 0x04B6, 0x0324, 0x0192,
};





//*******************************************************************************




static const s16 circle_limit_table[129]=
{
32641,32515,32391,32268,32147,32026,31908,31790,31674,31559,
31445,31333,31221,31111,31002,30894,30788,30682,30578,30474,
30372,30270,30170,30070,29972,29875,29778,29682,29588,29494,
29401,29309,29218,29128,29038,28949,28862,28775,28688,28603,
28518,28434,28351,28268,28186,28105,28025,27945,27866,27788,
27710,27633,27556,27481,27405,27331,27257,27184,27111,27039,
26967,26896,26825,26755,26686,26617,26549,26481,26414,26347,
26281,26215,26150,26085,26020,25957,25893,25830,25768,25706,
25644,25583,25522,25462,25402,25343,25284,25225,25167,25109,
25052,24995,24938,24882,24826,24771,24716,24661,24607,24553,
24499,24446,24393,24340,24288,24236,24184,24133,24082,24031,
23981,23931,23881,23832,23783,23734,23685,23637,23589,23542,
23494,23447,23401,23354,23308,23262,23216,23171,23126
};


static const s16 circle_limit_table2[87]=
{
32494,32360,32096,31839,31587,31342,31102,30868,30639,30415,\
30196,29981,29771,29565,29464,29265,29069,28878,28690,28506,\
28325,28148,27974,27803,27635,27470,27309,27229,27071,26916,\
26764,26614,26467,26322,26180,26039,25901,25766,25632,25500,\
25435,25307,25180,25055,24932,24811,24692,24574,24458,24343,\
24230,24119,24009,23901,23848,23741,23637,23533,23431,23331,\
23231,23133,23036,22941,22846,22753,22661\
};


//s16 BUFFER_Current_Quadrature_Acquisitions[128];

//s16 BUFFER_Current_Direct_Acquisitions[128];

s16 BUFFER_Beta_Voltage[SAMPLES];



s16 BUFFER_Beta_Acquisitions[SAMPLES];

s16 qValpha=0;
s16 qVbeta=0;

//u8 Index_Sin=0;
u16 Index_Sin=0;
//u8 Index_Cos=0;
u16 Index_Cos=0;
u8 Index_Beta=0;

u16 Theta_Grid=0;

s16 Theta=0;
u16 Theta_previous=0;
u16 Theta_time=0;
s16 Cos_Theta=0;
s16 Sin_Theta=0;
s16 Beta=0;

s16 AVG_Quadrature_Current=0;
s16 AVG_Direct_Current=0;

s16 AVG_BUS_DC=0;
s16 AVG_Current_DC=0;

s16 AVG_Alpha_Current=0;
s16 AVG_Beta_Current=0;

extern s16 Output_qVd_Grid;
s16 Output_qId_Inverter=0;
s16 Output_qIq_Inverter=0;
s16 Output_PID_Reactive_Power=0;
s16 Output_PID_Active_Power=0;

Volt_Components Grid_Volt_q_d;            // Vq & Vd, voltages on a reference
                                  // frame synchronous with the Grid_Voltage*/
Curr_Components     Inverter_q_d;

extern SystStatus_t State_Control;//default

extern bool MPPT_EN;

u16 zero_detect=0;

u16 VqFiltered=0;
u16 VqFiltered_max=0;
u16 VqFiltered_min=65500;
u32 VqFiltered_mean=0;
u16 VqFiltered_prec=0;
extern u8 MPPT_num;
extern bool Vacprot;

/*******************************************************************************
* Function Name  : DQ_PLL_Grid
* Description    : The 2-axis orthogonal system are converted into a 2-axis time invariant RF
*                  Vd=Valpha*Cos(Theta)+Vbeta*Sin(Theta) prima era valpha cos +Vbetasen
*                  Vq=-Valpha*Sin(Theta)+Vbeta*Cos(Theta)
*
* Input          :  qValpha, qVbeta.
* Output         : w,Theta.
* Return         : none.
*******************************************************************************/
Volt_Components DQ_PLL_Grid(s16 Voltage)

{

  Volt_Components Volt_Output;

  s32 qVd_Grid_tmp_1=0, qVd_Grid_tmp_2=0;
  s32 qVq_Grid_tmp_1=0, qVq_Grid_tmp_2=0;

  s16 qVd_Grid_1=0,qVd_Grid_2=0;
  s16 qVq_Grid_1=0,qVq_Grid_2=0;

  static u8 Count_Beta_2=0;
  static u8 Count_Alpha_2=0;

  qValpha=Voltage;

  Count_Alpha_2=(u8)(Count_Beta_2+Delay90); //((1/50Hz)/(1/21.600kHz)/4)-256
  //Count_Alpha_2=(u8)(Count_Beta_2+141);
  BUFFER_Beta_Voltage[Count_Beta_2]=qValpha;

  qVbeta=-BUFFER_Beta_Voltage[(u8)Count_Alpha_2];

  Count_Beta_2++;

//  Index_Sin=((u8)(Theta>>8));
//  Index_Cos =((u8)(Theta>>8)+64);

  Index_Sin = ((u16)(Theta>>7) & (u16)(0x01FF));
  Index_Cos = ((u16)(Theta>>7)+128) & (u16)(0x01FF);

//  Index_Sin = (u16)(new_mul_q15_q15_q31(Theta, 350) >> 16);
//  Index_Cos = (u16)(new_mul_q15_q15_q31(Theta, 350) >> 16)+88;

  Sin_Theta = Sin_Cos_Table[Index_Sin];
  Cos_Theta = Sin_Cos_Table[Index_Cos];


  mul_q15_q15_q31(qValpha,Cos_Theta,&qVq_Grid_tmp_1);
  mul_q15_q15_q31(qVbeta,Sin_Theta,&qVq_Grid_tmp_2);


  qVq_Grid_1 = (s16)(qVq_Grid_tmp_1/65536);
  qVq_Grid_2 = (s16)(qVq_Grid_tmp_2/65536);


  Volt_Output.qV_Quadrature = (s16)((qVq_Grid_1)-(qVq_Grid_2));	//SECONDO NREL


  mul_q15_q15_q31(qValpha,Sin_Theta,&qVd_Grid_tmp_1);           // SECONDO NREL
  mul_q15_q15_q31(qVbeta,Cos_Theta,&qVd_Grid_tmp_2);            // SECONDO NREL


  qVd_Grid_1 = ((s16)(qVd_Grid_tmp_1/65536));
  qVd_Grid_2 = ((s16)(qVd_Grid_tmp_2/65536));

  Volt_Output.qV_Direct = (s16)((qVd_Grid_1)+(qVd_Grid_2));	  //Vd component




  return (Volt_Output);


}

/*******************************************************************************
* Function Name  : SimpleMovingAvarage
* Description    :
* Input          :  Volt_Output_nofiltered.
* Output         : average
* Return         : average
*******************************************************************************/
s16 SimpleMovingAvarage_direct(s16 Volt_Output_nofiltered)
 {
  static u16 number_of_samples = NUMBEROFPOINT;
  static u16 freq_cuttoff = CUTTOFREQ;    //in Hz
  static u16 freq_sampling = SAMPLINGFREQ; //in Hz
  static u16 index_int=0;
  static s16 average=0;
  static s32 average_sum=0;
  static s16 BUFFER_average[NUMBERS_OF_SAMPLES]={0};
  static bool average_flag_status=FALSE;
  static u16 yindex=0;
  static u16 index=0;

//  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

  if(index_int < (NUMBERS_OF_SAMPLES-1))
       BUFFER_average[index_int] = Volt_Output_nofiltered;

   if(index_int == (NUMBERS_OF_SAMPLES-1))
    {

      average_sum = average_sum - BUFFER_average[index]; //12uSec

      BUFFER_average[index]=Volt_Output_nofiltered;

      if(index < ( NUMBERS_OF_SAMPLES-1)) index++;
      else index = 0;

      average_sum = average_sum + Volt_Output_nofiltered; //12uSec

      average = (s16)(average_sum/(NUMBERS_OF_SAMPLES));
    }
   else
    {
     average_sum=0;

     for(yindex=0;yindex<=index_int;yindex++)
      {
        average_sum = average_sum + BUFFER_average[yindex];
      }
        average = (s16)(average_sum/(index_int+1));
        index_int++;
    }

//    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
    return (s16)average;
}

/*******************************************************************************
* Function Name  : SimpleMovingAvarage
* Description    :
* Input          :  Volt_Output_nofiltered.
* Output         : average
* Return         : average
*******************************************************************************/
s16 SimpleMovingAvarage_quadrature(s16 Volt_Output_nofiltered_quad)
 {
  static u16 number_of_samples = NUMBEROFPOINT;
  static u16 freq_cuttoff = CUTTOFREQ;    //in Hz
  static u16 freq_sampling = SAMPLINGFREQ; //in Hz
  static u16 index_int_quad =0;
  static s16 average_quad =0;
  static s32 average_sum_quad =0;
  static s16 BUFFER_average_quad[NUMBERS_OF_SAMPLES]={0};
  static u16 yindex_quad = 0;
  static u16 index_quad = 0;

//  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

  if(index_int_quad < (NUMBERS_OF_SAMPLES-1))
       BUFFER_average_quad[index_int_quad] = Volt_Output_nofiltered_quad;

   if(index_int_quad == (NUMBERS_OF_SAMPLES-1))
    {

      average_sum_quad = average_sum_quad - BUFFER_average_quad[index_quad]; //12uSec

      BUFFER_average_quad[index_quad] = Volt_Output_nofiltered_quad;

      if(index_quad < ( NUMBERS_OF_SAMPLES-1))
        index_quad++;
      else index_quad = 0;

      average_sum_quad = average_sum_quad + Volt_Output_nofiltered_quad; //12uSec

      average_quad = (s16)(average_sum_quad/(NUMBERS_OF_SAMPLES));
    }
   else
    {
     average_sum_quad=0;

     for(yindex_quad=0;yindex_quad<=index_int_quad;yindex_quad++)
      {
        average_sum_quad = average_sum_quad + BUFFER_average_quad[yindex_quad];
      }
        average_quad = (s16)(average_sum_quad/(index_int_quad+1));
        index_int_quad++;
    }

//    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);

    return (s16)average_quad;

}


/*******************************************************************************
* Function Name  : DQ_PLL_Grid
* Description    : The 2-axis orthogonal system are converted into a 2-axis time invariant RF
*                  Vd=Valpha*Cos(Theta)+Vbeta*Sin(Theta) prima era valpha cos +Vbetasen
*                  Vq=-Valpha*Sin(Theta)+Vbeta*Cos(Theta)
*
* Input          :  qValpha, qVbeta.
* Output         : w,Theta.
* Return         : none.
*******************************************************************************/
Volt_Components PLL_AntiTrasfPark(s16 Voltage,s16 Vbeta_anttrf)
{
  Volt_Components Volt_Output;


  s32 qVd_Grid_tmp_1=0, qVd_Grid_tmp_2=0;
  s32 qVq_Grid_tmp_1=0, qVq_Grid_tmp_2=0;

  s16 qVd_Grid_1=0,qVd_Grid_2=0;
  s16 qVq_Grid_1=0,qVq_Grid_2=0;

  static u8 Count_Beta_2=0;
  static u8 Count_Alpha_2=0;

  qValpha=Voltage;

  qVbeta=(s16)Vbeta_anttrf;

//  Count_Alpha_2=(u8)(Count_Beta_2+167);
//
//  BUFFER_Beta_Voltage[Count_Beta_2]=qValpha;
//
//  qVbeta=-BUFFER_Beta_Voltage[(u8)Count_Alpha_2];
//
//  Count_Beta_2++;

  Index_Sin = ((u16)(Theta>>7) & (u16)(0x01FF));
  Index_Cos = ((u16)(Theta>>7)+128) & (u16)(0x01FF);

  Sin_Theta = Sin_Cos_Table[Index_Sin];
  Cos_Theta = Sin_Cos_Table[Index_Cos];

  mul_q15_q15_q31(qValpha,Cos_Theta,&qVq_Grid_tmp_1);
  mul_q15_q15_q31(qVbeta,Sin_Theta,&qVq_Grid_tmp_2);


  qVq_Grid_1 = (s16)(qVq_Grid_tmp_1/65536);
  qVq_Grid_2 = (s16)(qVq_Grid_tmp_2/65536);


  Volt_Output.qV_Quadrature = (s16)((qVq_Grid_1)-(qVq_Grid_2));	//SECONDO NREL


  mul_q15_q15_q31(qValpha,Sin_Theta,&qVd_Grid_tmp_1);           // SECONDO NREL
  mul_q15_q15_q31(qVbeta,Cos_Theta,&qVd_Grid_tmp_2);            // SECONDO NREL


  qVd_Grid_1 = ((s16)(qVd_Grid_tmp_1/65536));
  qVd_Grid_2 = ((s16)(qVd_Grid_tmp_2/65536));

  Volt_Output.qV_Direct = (s16)((qVd_Grid_1)+(qVd_Grid_2));	  //Vd component

/************************************************************************************/
  Volt_Output.qV_Direct = (s16) SimpleMovingAvarage_direct(Volt_Output.qV_Direct);             //time=250nSec
  Volt_Output.qV_Quadrature = (s16) SimpleMovingAvarage_quadrature(Volt_Output.qV_Quadrature); //time=250nSec
/***********************************************************************************/

  return (Volt_Output);
}

Volt_AlphaBeta_Components SOGI(s16 gridvoltage,s16 omega_value)
{
  Volt_AlphaBeta_Components alfabeta_sogi_out;

  static u8 gain_K = 1;
  static s16 Integral_Term_SOGI_1 = 0;
  static s16 Integral_Term_SOGI_2 = 0;
  static s16 error_V_Valpha = 0;
  static s16 error_Valpha2_Vbeta = 0;
  static s16 product_x_omega = 0;
  static s16 product_x_omega_2 = 0;
  static s16 grid_max=0;
  static s16 grid_min=0;


//  error_V_Valpha = (s16)(gridvoltage - alfabeta_sogi_out.qValpha);
//
//  error_Valpha2_Vbeta = (s16)(error_V_Valpha - alfabeta_sogi_out.qVbeta);
//
//  product_x_omega = (s32) (error_Valpha2_Vbeta * omega_value);

  if(gridvoltage>=grid_max) grid_max = gridvoltage;

  if(gridvoltage<=grid_min) grid_min = gridvoltage;

  product_x_omega = (s16) (gridvoltage * 1);

  Integral_Term_SOGI_1 = (s16) (Integral_Term_SOGI_1 + product_x_omega);

  alfabeta_sogi_out.qValpha = (s16) (Integral_Term_SOGI_1);

//  DAC_SetChannel(DAC_CH_1,(u16)  gridvoltage); // COMMENTED OUT!!!!!!!!!!!!!!!!!!!!

//  product_x_omega_2 =  alfabeta_sogi_out.qValpha * omega_value;
//
//  Integral_Term_SOGI_2 = (s16) (Integral_Term_SOGI_2 + product_x_omega_2);
//
//  alfabeta_sogi_out.qVbeta = (s16) (Integral_Term_SOGI_2/65536);

  return (alfabeta_sogi_out);
}

Volt_AlphaBeta_Components SOGI_TEST(s16 gridvoltage,s16 omega_value)
{
  Volt_AlphaBeta_Components alfabeta_sogi_out;

  static u8 gain_K = 1;
  static s16 Integral_Term_SOGI_1 = 0;
  static s16 Integral_Term_SOGI_2 = 0;
  static s16 error = 0;

  static s16 product_x_omega = 0;
  static s16 product_x_omega_2 = 0;
  static s16 grid_max=0;
  static s16 grid_min=0;

  if(gridvoltage>=grid_max) grid_max = gridvoltage;

  if(gridvoltage<=grid_min) grid_min = gridvoltage;

  error = (s16)(gridvoltage - alfabeta_sogi_out.qValpha);


  alfabeta_sogi_out.qValpha += (s16)(((error*gain_K-(alfabeta_sogi_out.qVbeta))*omega_value)*SAMPLING_TIME) ;


  alfabeta_sogi_out.qVbeta += (s16)((alfabeta_sogi_out.qValpha)*omega_value*SAMPLING_TIME);

  return (alfabeta_sogi_out);
}

/*******************************************************************************
* Function Name  : DQ_PLL_Grid
* Description    : The 2-axis orthogonal system are converted into a 2-axis time invariant RF
*                  Vd=Valpha*Cos(Theta)+Vbeta*Sin(Theta) prima era valpha cos +Vbetasen
*                  Vq=-Valpha*Sin(Theta)+Vbeta*Cos(Theta)
*
* Input          :  qValpha, qVbeta.
* Output         : w,Theta.
* Return         : none.
*******************************************************************************/
Volt_Components DQ_PLL_Grid_x_SOGI(s16 alpha,s16 beta)
{

  Volt_Components Volt_Output;

  s32 qVd_Grid_tmp_1=0, qVd_Grid_tmp_2=0;
  s32 qVq_Grid_tmp_1=0, qVq_Grid_tmp_2=0;

  s16 qVd_Grid_1=0,qVd_Grid_2=0;
  s16 qVq_Grid_1=0,qVq_Grid_2=0;

  qValpha = alpha;

  qVbeta = beta;

//  Index_Sin=((u8)(Theta>>8));
//  Index_Cos =((u8)(Theta>>8)+64);

  Index_Sin = ((u16)(Theta>>7) & (u16)(0x01FF));
  Index_Cos = ((u16)(Theta>>7)+128) & (u16)(0x01FF);

//  Index_Sin = (u16)(new_mul_q15_q15_q31(Theta, 350) >> 16);
//  Index_Cos = (u16)(new_mul_q15_q15_q31(Theta, 350) >> 16)+88;

  Sin_Theta = Sin_Cos_Table[Index_Sin];
  Cos_Theta = Sin_Cos_Table[Index_Cos];


  mul_q15_q15_q31(qValpha,Cos_Theta,&qVq_Grid_tmp_1);
  mul_q15_q15_q31(qVbeta,Sin_Theta,&qVq_Grid_tmp_2);


  qVq_Grid_1 = (s16)(qVq_Grid_tmp_1/65536);
  qVq_Grid_2 = (s16)(qVq_Grid_tmp_2/65536);


  Volt_Output.qV_Quadrature = (s16)((qVq_Grid_1)-(qVq_Grid_2));	//SECONDO NREL


  mul_q15_q15_q31(qValpha,Sin_Theta,&qVd_Grid_tmp_1);           // SECONDO NREL
  mul_q15_q15_q31(qVbeta,Cos_Theta,&qVd_Grid_tmp_2);            // SECONDO NREL


  qVd_Grid_1 = ((s16)(qVd_Grid_tmp_1/65536));
  qVd_Grid_2 = ((s16)(qVd_Grid_tmp_2/65536));

  Volt_Output.qV_Direct = (s16)((qVd_Grid_1)+(qVd_Grid_2));	  //Vd component

  return (Volt_Output);
}



/*******************************************************************************
* Function Name  : DQ_Current_Inverter
* Description    : The 2-axis orthogonal system are converted into a 2-axis time invariant RF
*               // Iq=-Ialpha_tmp*Sin(Theta)+Ibeta_tmp*Cos(Theta)
*                  //  Id=Ialpha_tmp*Cos(theta)+Ibeta_tmp*Sin(Theta)
*
* Input          :  qIalpha, qIbeta.
* Output         : w,Theta.
* Return         : none.
*******************************************************************************/
Curr_Components DQ_Current_Inverter(s16 qIalpha,s16 qIbeta)

{

  Curr_Components Curr_Output_Inverter;

  s32 qId_Inverter_tmp_1=0, qId_Inverter_tmp_2=0;
  s32 qIq_Inverter_tmp_1=0, qIq_Inverter_tmp_2=0;
//  s32 qBeta_Inverter_tmp=0;

 // static u8 Index=0;////  to be deleted if not used!!!

  s16 qId_Inverter_1=0,qId_Inverter_2=0;
  s16 qIq_Inverter_1=0,qIq_Inverter_2=0;

  mul_q15_q15_q31(qIalpha,Sin_Theta,&qId_Inverter_tmp_1);
  mul_q15_q15_q31(qIbeta,Cos_Theta,&qId_Inverter_tmp_2) ;

  qId_Inverter_1 = (s16)(qId_Inverter_tmp_1/65536);
  qId_Inverter_2 = (s16)(qId_Inverter_tmp_2/65536);

  Curr_Output_Inverter.qI_Direct = (s16)((qId_Inverter_1)+(qId_Inverter_2));


  mul_q15_q15_q31(qIalpha,Cos_Theta,&qIq_Inverter_tmp_1);
  mul_q15_q15_q31(qIbeta,Sin_Theta,&qIq_Inverter_tmp_2);


  qIq_Inverter_1 = ((s16)(qIq_Inverter_tmp_1/65536));
  qIq_Inverter_2 = ((s16)(qIq_Inverter_tmp_2/65536));

  Curr_Output_Inverter.qI_Quadrature = (s16)((qIq_Inverter_1)-(qIq_Inverter_2));

  return (Curr_Output_Inverter);

}


/*********************************Inverse Park Transformation *****************/
/**************Valfa=Vd*Cos(theta)-Vq*Sin(theta)*******************************/
/**************Vbeta=Vq*Cos(theta)+Vd*Sin(theta)  *****************************/
/******************************************************************************/
Volt_AlphaBeta_Components Rev_Park(s16 Output_QuadraturePID,s16 Output_DirectPID)
{
  s32 qValpha_tmp1,qValpha_tmp2=0;    //qVbeta_tmp1,qVbeta_tmp2;
  s16 qValpha_1,qValpha_2=0;          //qVbeta_1,qVbeta_2;
  Volt_AlphaBeta_Components Volt_Output;



   mul_q15_q15_q31(Output_DirectPID,Sin_Theta,&qValpha_tmp1);

   mul_q15_q15_q31(Output_QuadraturePID,Cos_Theta,&qValpha_tmp2);


  qValpha_1 = (s16)(qValpha_tmp1/65536);
  qValpha_2 = (s16)(qValpha_tmp2/65536);

  Volt_Output.qValpha = ((qValpha_1)+(qValpha_2));

  Volt_Output.qVbeta = 0;//((qVbeta_1)+(qVbeta_2));

  return(Volt_Output);
}


/*********************************Inverse Park Transformation *****************/
/**************Valfa=Vd*Cos(theta)-Vq*Sin(theta)*******************************/
/**************Vbeta=Vq*Cos(theta)+Vd*Sin(theta)  *****************************/
/******************************************************************************/
Volt_AlphaBeta_Components Rev_Park_newPLL(s16 Output_QuadraturePID,s16 Output_DirectPID)
{
  s32 qVbeta_tmp1,qVbeta_tmp2=0;    //qVbeta_tmp1,qVbeta_tmp2;
  s16 qVbeta_1,qVbeta_2=0;          //qVbeta_1,qVbeta_2;
  Volt_AlphaBeta_Components Volt_Output;


   mul_q15_q15_q31(Output_QuadraturePID,Sin_Theta,&qVbeta_tmp1);

   mul_q15_q15_q31(Output_DirectPID,Cos_Theta,&qVbeta_tmp2);


  qVbeta_1 = (s16)(qVbeta_tmp1/65536);
  qVbeta_2 = (s16)(qVbeta_tmp2/65536);

  Volt_Output.qValpha = 0;

  Volt_Output.qVbeta =  ((qVbeta_2)-(qVbeta_1));

  return(Volt_Output);
}


void Calc_Theta_Grid(s16 Input_Integration)
{
 static s32 Delta_Theta_tmp=0;
 static s16 Delta_Theta=0;
 static u16 freq_monitor_time=0;

 mul_q15_q15_q31(Input_Integration,SAMPLING_TIME,&Delta_Theta_tmp);

 Delta_Theta = (s16)(Delta_Theta_tmp>>16);

 Theta=Theta+Delta_Theta + 0;//


 Theta_time++;

 if(tti<4096)
 {
 tti++;
 theta_total[tti] = (int32_t)(((Theta+0x8000)-Theta_previous));//Theta; //(int32_t)(((Theta+0x8000)-Theta_previous));
 }


  if(zero_detect==50)
 {
   VqFiltered_min=65500;
   VqFiltered_max=0;
   Theta_time=0;
   zero_detect=0;
 }

 VqFiltered_prec = (u16)(Grid_Volt_q_d.qV_Quadrature + 0x8000);

 VqFiltered = (u16)(((s32)(((s32)VqFiltered<<8) - (s32)VqFiltered) +  (s32)(VqFiltered_prec))>>8);

  if(VqFiltered>VqFiltered_max) VqFiltered_max=VqFiltered;
  else if(VqFiltered<VqFiltered_min) VqFiltered_min=VqFiltered;

  VqFiltered_mean=(u16)((VqFiltered_max+VqFiltered_min)>>1);
/*
  if (ii<2048)
  {
  ii++;
	buffer_1[ii]=VqFiltered_mean;
	buffer_2[ii]=VqFiltered_prec;
  }
if(ii>2047)
{

}
*/
  //GPIOG -> ODR ^= (1 << 9);

  //265V max & 186V min
   if((VqFiltered_mean>=48641 || VqFiltered_mean<=32837) && State_Control==GRID_INSERTION && MPPT_EN==TRUE && zero_detect>5 && Vacprot==TRUE)
   {
	   LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
       State_Control = STOP_WITH_DELAY;
       Diagnostic_Control=GRID_VOLTAGE_OUT_OF_RANGE;
   }

   //if((Theta_time==((Theta_Grid/4)-REL_ON_TICK) || Theta_time==((Theta_Grid*3/4)-REL_ON_TICK)))

  // {
	   //LL_GPIO_TogglePin(SD_CS_GPIO_Port, SD_CS_Pin);
  // }

 //************************** ISLANDING DETECTION *****************************
 if(((Theta+0x8000)-Theta_previous)<=-20000)
 {
  zero_detect++;

  Theta_Grid = Theta_time;

//****************** DIAGNOSTIC AC - VOLTAGE AND FREQ AT STARTUP ***************
  //Freq 47 Hz - 53 Hz
  //Vac 185V - 265V
  if((State_Control==DIAGNOSTIC_AC_LINE) || (State_Control==DIAGNOSTIC_DC_LINE))
  {
    if((Theta_time>(GRID_FREQ_MAX) && Theta_time<(GRID_FREQ_MIN)) && (VqFiltered_mean >= 32837 && VqFiltered_mean<=48641)) //48641 = 254V
    {
      //36960 e 38830
      if(freq_monitor_time>=50) //1 sec
      {
        GDVoltage = GRID_VOLTAGE_INSIDE_RANGE;
        Freq_Control = FREQ_INSIDE_RANGE; //inside the G83 freq range
        freq_monitor_time=0;
      }
      if(Freq_Control==FREQ_OUT_OF_RANGE && GDVoltage == GRID_VOLTAGE_OUT_OF_RANGE)
         freq_monitor_time++;
     }
    else
     {
       freq_monitor_time=0;
       Grid_Voltage_max=0;
       Grid_Voltage_min=0;

       Freq_Control = FREQ_OUT_OF_RANGE;
       GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
     }
   }
//****************** DIAGNOSTIC AC - VOLTAGE AND FREQ AT GRID CONNECTION *******
  else
  {
	  if(State_Control == GRID_INSERTION && MPPT_EN == TRUE)
	  {



	   if((Theta_time<=GRID_FREQ_MAX || Theta_time>=GRID_FREQ_MIN))
		 {
		   //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
		   State_Control = STOP_WITH_DELAY;
		   Diagnostic_Control=FREQ_OUT_OF_RANGE;
		   Freq_Control = FREQ_OUT_OF_RANGE;
		 }
	   if((VqFiltered_mean <= 32837 && VqFiltered_mean >= 48641))
		 {
		   //LL_GPIO_ResetOutputPin(RELAY_GRID_GPIO_Port, RELAY_GRID_Pin);
		   State_Control = STOP_WITH_DELAY;
		   Diagnostic_Control= GRID_VOLTAGE_OUT_OF_RANGE;
		   GDVoltage = GRID_VOLTAGE_OUT_OF_RANGE;
		 }
	  }
  }
//****************** DIAGNOSTIC AC - VOLTAGE AND FREQ AT GRID CONNECTION *******


    Theta_time=0;
 }

 Theta_previous=Theta+0x8000;





 //********************** END ISLANDING DETECTION ******************************

}

/*******************************************************************************
* Function Name  :   DQ_Power_Estimation
* Description    : Estimate the reactive power in the DQ reference frame

* Input          : None
* Output         : Power_Component
* Return         : None
*******************************************************************************/


Power_Components DQ_Power_Estimation(Curr_Components QD_Current)
{
   Power_Components Power_Output;

   s32 P_Active_tmp_1,P_Active_tmp_2=0;
   s32 Q_Reactive_tmp_1,Q_Reactive_tmp_2=0;

   s16 P_Active_1,P_Active_2=0;
   s16 Q_Reactive_1,Q_Reactive_2=0;



   mul_q15_q15_q31(Grid_Volt_q_d.qV_Direct,QD_Current.qI_Direct,&P_Active_tmp_1) ;
   mul_q15_q15_q31(Grid_Volt_q_d.qV_Quadrature,QD_Current.qI_Quadrature,&P_Active_tmp_2);

  P_Active_1 = (s16)(P_Active_tmp_1/65536);
  P_Active_2 = (s16)(P_Active_tmp_2/65536);

Power_Output.P_Active = (s16)((P_Active_1)+(P_Active_2));

  mul_q15_q15_q31(Grid_Volt_q_d.qV_Direct,QD_Current.qI_Quadrature,&Q_Reactive_tmp_1) ;
  mul_q15_q15_q31(Grid_Volt_q_d.qV_Quadrature,QD_Current.qI_Direct,&Q_Reactive_tmp_2);


  Q_Reactive_1 = (s16)(Q_Reactive_tmp_1/65536);
  Q_Reactive_2 = (s16)(Q_Reactive_tmp_2/65536);

  //Power_Output.Q_Reactive = (s16)((Q_Reactive_1)+(Q_Reactive_2));

  Power_Output.Q_Reactive = (s16)(-(Q_Reactive_1)+(Q_Reactive_2));
  return (Power_Output);


}




/*******************************************************************************
* Function Name  : CrossDecoupling_Control
* Description    : Achieve a decoupling control of Q and D axis

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



void CrossDecoupling_Control(void)
{

  s32 Omega_L_tmp, Omega_L_Quadrature_tmp,Omega_L_Direct_tmp=0;
  s16 Omega_L,Omega_L_Direct,Omega_L_Quadrature=0;

  mul_q15_q15_q31(INDUCTANCE_VALUE,Output_qVd_Grid,&Omega_L_tmp);
  Omega_L = (s16)(Omega_L_tmp>>16);

  mul_q15_q15_q31(Omega_L,Inverter_q_d.qI_Direct,&Omega_L_Direct_tmp);
  mul_q15_q15_q31(Omega_L,Inverter_q_d.qI_Quadrature,&Omega_L_Quadrature_tmp);

  Omega_L_Direct =    (s16)(Omega_L_Direct_tmp>>16);
  Omega_L_Quadrature =(s16)(Omega_L_Quadrature_tmp>>16);

   Output_qIq_Inverter=(s16)(Output_qIq_Inverter-Omega_L_Direct);//+(Grid_Volt_q_d.qV_Quadrature));//+Grid_Volt_q_d.qV_Quadrature);//);//Omega_L_Direct);//
   Output_qId_Inverter=(s16)(Output_qId_Inverter+Omega_L_Quadrature);//+(Grid_Volt_q_d.qV_Direct));//+Grid_Volt_q_d.qV_Direct);//-Omega_L_Quadrature);    // -Omega_L_Quadrature);//+Grid_Volt_q_d.qV_Direct

}


/*******************************************************************************
* Function Name  : RevPark_Circle_Limitation
* Description    : Check if Stat_Volt_q_d.qV_Component1^2 + Stat_Volt_q_d.qV_Component2^2 <= 32767^2
                   Apply limitation if previous condition is not met,
                   by keeping a constant ratio
                   Stat_Volt_q_d.qV_Component1/Stat_Volt_q_d.qV_Component2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



//void RevPark_Circle_Limitation(void)
//
//{
//   u32 uw_temp;
//   s32 sw_temp;
//
//   sw_temp =(s32)(Output_qIq_Inverter) * Output_qIq_Inverter +
//     (s32)(Output_qId_Inverter) * Output_qId_Inverter;
//
//   uw_temp =(u32) sw_temp;
//  /* uw_temp min value 0, max value 2*32767*32767 */
//
//  if (uw_temp > (u32)(32111 * 32111))
//  {
//    u16 hTable_Element;
//
//    uw_temp /= (u32)(16777216);
//    /* wtemp min value CLASS_PARAMS->bStart_index, max value 127 */
//    uw_temp -= 61;
//
//    /* uw_temp min value 0, max value 127 - CLASS_PARAMS->bStart_index */
//    hTable_Element = circle_limit_table2[(u8)uw_temp];
//
//    sw_temp = Output_qIq_Inverter * (s32)hTable_Element;
//    Output_qIq_Inverter = (s16)(sw_temp/32768);
//
//    sw_temp = Output_qId_Inverter * (s32)(hTable_Element);
//    Output_qId_Inverter = (s16)(sw_temp/32768);
//  }
//}

void RevPark_Circle_Limitation(void)
{
s32 temp;

temp = Output_qIq_Inverter * Output_qIq_Inverter
             + Output_qId_Inverter * Output_qId_Inverter;  // min value 0, max value 2*32767*32767

if ( temp > (32767*32767) )       // temp > 32767*32767,
                                  // & temp <= 2*32767*32767
   {
   unsigned long long temp2;
   u16 index;

   //temp2 = (unsigned long long ) 128*temp;
   //temp2 /= (32767*32767);   // min value 128, max value 256
   temp2 = (unsigned long long )257*temp;
   temp2 /= (u32)(32768*32768);   // min value 128, max value 256
   temp = temp2/2;
   temp -= 128;   // min value 0, max value 128
   index = circle_limit_table[(u8)temp];

   temp = (s16)Output_qIq_Inverter * (u16)(index);
   Output_qIq_Inverter = (s16)(temp/32768);

   temp = (s16)Output_qId_Inverter * (u16)(index);
   Output_qId_Inverter = (s16)(temp/32768);
   }
}

/*******************************************************************************
* Function Name  : RevPark_Circle_Limitation
* Description    : Check if Stat_Volt_q_d.qV_Component1^2 + Stat_Volt_q_d.qV_Component2^2 <= 32767^2
                   Apply limitation if previous condition is not met,
                   by keeping a constant ratio
                   Stat_Volt_q_d.qV_Component1/Stat_Volt_q_d.qV_Component2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



Volt_Components RevPark_Circle_Limitation_DQ_PLL(s16 Output_QuadraturePID,s16 Output_DirectPID) // NIC NIE ROBI

{
   u32 uw_temp;
   s32 sw_temp;

   Volt_Components Volt_Output_limit;

   sw_temp =(s32)(Output_QuadraturePID) * Output_QuadraturePID +
     (s32)(Output_DirectPID) * Output_DirectPID;

   uw_temp =(u32) sw_temp;
  /* uw_temp min value 0, max value 2*32767*32767 */

  if (uw_temp > (u32)(32111 * 32111))
  {
    u16 hTable_Element;

    uw_temp /= (u32)(16777216);
    /* wtemp min value CLASS_PARAMS->bStart_index, max value 127 */
    uw_temp -= 61;

    /* uw_temp min value 0, max value 127 - CLASS_PARAMS->bStart_index */
    hTable_Element = circle_limit_table2[(u8)uw_temp];

    sw_temp = Output_QuadraturePID * (s32)hTable_Element;
    Volt_Output_limit.qV_Quadrature = (s16)(sw_temp/32768);

    sw_temp = Output_DirectPID * (s32)(hTable_Element);
    Volt_Output_limit.qV_Direct = (s16)(sw_temp/32768);

    return (Volt_Output_limit);
  }
}


/*******************************************************************************
* Function Name  : BUS_DC_Filtering
* Description    : Apply a digital filtering calculating the average on xx samples performing it every loop.


* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



s16 BUS_DC_Filtering(s16 Bus_Voltage_Input, s16 samples) // NIC NIE ROBI

{
  static s32 Sum_Voltage=0;

  static u16 samples_init = 0;

  //s16 Output_S16=0;

   if (samples_init < samples)
	   {
		   samples_init++;

		   Sum_Voltage=(s32)((Sum_Voltage+Bus_Voltage_Input)-(AVG_BUS_DC));


		   AVG_BUS_DC = ((s16)(Sum_Voltage/samples)); //  To be  defined the maximum number of samples

		   return 0;
	   }

   else
	   {
	   	   return AVG_BUS_DC;
	   }


 }




s16 Current_DC_Filtering(s16 DC_Current_Input)

{
  static s32 Sum_Current=0;

  //s16 Output_S16=0;


   Sum_Current=(s32)((Sum_Current+DC_Current_Input)-(AVG_Current_DC));


   AVG_Current_DC = ((s16)(Sum_Current/256)); //  To be  defined the maximum number of samples

   return AVG_Current_DC;

 }





/*******************************************************************************
* Function Name  : DQ_Filtering
* Description    : Apply a digital filtering calculating the average on 16 samples performing it every loop.
                   after a charge phase necessary to fill the buffer

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



Curr_Components DQ_Filtering(Curr_Components QD_Current) //// 10 próbek / 25khz/10 = 2.5khz filter zero -> 250hz HIT

{

	Curr_Components AVG_Current;

  static s32 Sum_Quadrature=0;
  static s32 Sum_Direct=0;


   Sum_Quadrature=(s32)((Sum_Quadrature+QD_Current.qI_Quadrature)-(AVG_Quadrature_Current));
   Sum_Direct=(s32)((Sum_Direct+QD_Current.qI_Direct)-(AVG_Direct_Current));

   AVG_Quadrature_Current=(s16)((Sum_Quadrature/8));
   AVG_Direct_Current=(s16)((Sum_Direct/8));

   AVG_Current.qI_Direct = AVG_Direct_Current;
   AVG_Current.qI_Quadrature = AVG_Quadrature_Current;

   return AVG_Current;

 }



/*******************************************************************************
* Function Name  : AlphaBeta_Filtering
* Description    : Apply a digital filtering calculating the average on 16 samples performing it every loop.
                   after a charge phase necessary to fill the buffer

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/



void AlphaBeta_Filtering(s16 Ialpha, s16 Ibeta) //// NIC NIE ROBI

{

  static s32 Sum_Alpha=0;
  static s32 Sum_Beta=0;


   Sum_Alpha=(s32)((Sum_Alpha+Ialpha)-(AVG_Alpha_Current));
   Sum_Beta=(s32)((Sum_Beta+Ibeta)-(AVG_Beta_Current));

   AVG_Alpha_Current=(s16)((Sum_Alpha/5));
   AVG_Beta_Current=(s16)((Sum_Beta/5));
 }


/*******************************************************************************
* Function Name  : Generate_90Degrees_Delay
* Description    : generates a 90 degrees phase shift for beta component

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

s16 Generate_90Degrees_Delay( s16 Alpha_Component)

{

  static u8 Count_Beta=0;
  static u8 Count_Alpha=0;

  s16 Output=0;

  Count_Alpha=(u8)((Count_Beta+Delay90));// for 90 degrees 36 //((1/50Hz)/(1/20.600kHz)/4)-256
  //Count_Alpha=(u8)((Count_Beta+141));// for 90 degrees 36
  BUFFER_Beta_Acquisitions[Count_Beta]=Alpha_Component;

  Output=-(BUFFER_Beta_Acquisitions[(u8)Count_Alpha]);// to compensate offset

  Count_Beta++;

  return ((s16)(Output));


}

/*******************************************************************************
* Function Name  : Generate_90Degrees_Delay
* Description    : generates a 90 degrees phase shift for beta component

* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

s16 Generate_10Degrees_Delay( s16 Alpha_Component)

{

  static u8 Count_Beta=0;
  static u8 Count_Alpha=0;

  s16 Output=0;

  Count_Alpha=(u8)((Count_Beta+Delay10));// for 90 degrees 36 //((1/50Hz)/(1/20.600kHz)/4)-256
  //Count_Alpha=(u8)((Count_Beta+141));// for 90 degrees 36
  BUFFER_Beta_Acquisitions[Count_Beta]=Alpha_Component;

  Output=-(BUFFER_Beta_Acquisitions[(u8)Count_Alpha]);// to compensate offset

  Count_Beta++;

  return ((s16)(Output));


}
/*******************************************************************************
* Function Name  : Check_BUS_DC
* Description    : check the BUS DC status

* Input          : TBD
* Output         : TBD
* Return         : TBD
*******************************************************************************/
void Check_BUS_DC (s16 Bus_Voltage_Input) // NIC NIE ROBI

{
 // static SystStatus_t Output;

   if (((Bus_Voltage_Input>20000)&&(Bus_Voltage_Input<=S16_MAX))|| ((AVG_BUS_DC>=100)&&(AVG_BUS_DC<=5000)))//DC_BUS_THRESHOLD)

   {

    State_Control=BUS_FAULT;

      }
}

/*******************************************************************************
* Function Name  : SimpleMovingAvarage
* Description    :
* Input          :  Volt_Output_nofiltered.
* Output         : average
* Return         : average
*******************************************************************************/
s16 SimpleMovingAvarage_DC_BUS(s16 Volt_Output_nofiltered_DC_BUS)
 {
  static u16 number_of_samples = NUMBEROFPOINT;
  static u16 freq_cuttoff = CUTTOFREQ;    //in Hz
  static u16 freq_sampling = SAMPLINGFREQ; //in Hz
  static u16 index_int_DC_BUS =0;
  static s16 average_DC_BUS =0;
  static s32 average_sum_DC_BUS =0;
  static s16 BUFFER_average_DC_BUS[samples_DC_BUS]={0};
  static u16 yindex_DC_BUS = 0;
  static u16 index_DC_BUS = 0;

//  GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);

  if(index_int_DC_BUS < (samples_DC_BUS-1))
       BUFFER_average_DC_BUS[index_int_DC_BUS] = Volt_Output_nofiltered_DC_BUS;

   if(index_int_DC_BUS == (samples_DC_BUS-1))
    {

      average_sum_DC_BUS = average_sum_DC_BUS - BUFFER_average_DC_BUS[index_DC_BUS]; //12uSec

      BUFFER_average_DC_BUS[index_DC_BUS] = Volt_Output_nofiltered_DC_BUS;

      if(index_DC_BUS < ( samples_DC_BUS-1))
        index_DC_BUS++;
      else index_DC_BUS = 0;

      average_sum_DC_BUS = average_sum_DC_BUS + Volt_Output_nofiltered_DC_BUS; //12uSec

      average_DC_BUS = (s16)(average_sum_DC_BUS/(samples_DC_BUS));
    }
   else
    {
     average_sum_DC_BUS=0;

     for(yindex_DC_BUS=0;yindex_DC_BUS<=index_int_DC_BUS;yindex_DC_BUS++)
      {
        average_sum_DC_BUS = average_sum_DC_BUS + BUFFER_average_DC_BUS[yindex_DC_BUS];
      }
        average_DC_BUS = (s16)(average_sum_DC_BUS/(index_int_DC_BUS+1));
        index_int_DC_BUS++;
    }

//    GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);

    return (s16)average_DC_BUS;

}
