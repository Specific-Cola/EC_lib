//=====================================================================================================
// Buzzer_def.h
//=====================================================================================================
//
//       IRobot  EC_lib
//
// author: @ dji
// modify: @ Specific_Cola
// 
//
//=====================================================================================================
#ifndef BUZZER_DEFINE_H__
#define BUZZER_DEFINE_H__

#include "struct_typedef.h"

#define BUZZER_C_MAJOR_OFFSET			24
#define BUZZER_A_MAJOR_OFFSET			BUZZER_C_MAJOR_OFFSET-BUZZER_OCTAVE_OFFSET+BUZZER_LA_OFFSET

#define BUZZER_DO_OFFSET				0
#define BUZZER_DO_SHARP_OFFSET			1
#define BUZZER_RE_OFFSET				2
#define BUZZER_RE_SHARP_OFFSET			3
#define BUZZER_MI_OFFSET				4
#define BUZZER_FA_OFFSET				5
#define BUZZER_FA_SHARP_OFFSET			6
#define BUZZER_SO_OFFSET				7
#define BUZZER_SO_SHARP_OFFSET			8
#define BUZZER_LA_OFFSET				9
#define BUZZER_LA_SHARP_OFFSET			10
#define BUZZER_SI_OFFSET				11

#define BUZZER_OCTAVE_OFFSET			12

#define BUZZER_GAP						INT16_MAX



#endif // !BUZZER_H__

