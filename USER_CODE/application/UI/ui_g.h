//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_g_Dynamic_Leg_Left_Angle;
extern ui_interface_number_t *ui_g_Dynamic_Leg_Right_Angle;
extern ui_interface_number_t *ui_g_Dynamic_Vy_Speed;
extern ui_interface_number_t *ui_g_Dynamic_Speed;
extern ui_interface_number_t *ui_g_Dynamic_Stair_State_Text;
extern ui_interface_number_t *ui_g_Dynamic_Cus_State_Text;
extern ui_interface_number_t *ui_g_Dynamic_RE_State_Text;
extern ui_interface_number_t *ui_g_Dynamic_Joy_State_Text;
extern ui_interface_number_t *ui_g_Dynamic_Pitch1_Angle;
extern ui_interface_number_t *ui_g_Dynamic_Pitch2_Angle;

void ui_init_g_Dynamic();
void ui_update_g_Dynamic();
void ui_remove_g_Dynamic();

extern ui_interface_line_t *ui_g_Static_Star_2;
extern ui_interface_line_t *ui_g_Static_Star_3;
extern ui_interface_line_t *ui_g_Static_Star_4;
extern ui_interface_line_t *ui_g_Static_Star_5;
extern ui_interface_line_t *ui_g_Static_Star_6;
extern ui_interface_string_t *ui_g_Static_Leg_Right_Text;
extern ui_interface_string_t *ui_g_Static_Vy_Text;
extern ui_interface_string_t *ui_g_Static_WUST_RM;
extern ui_interface_string_t *ui_g_Static_Star_Text_1;
extern ui_interface_string_t *ui_g_Static_Star_Text_2;
extern ui_interface_string_t *ui_g_Static_Star_Text_3;
extern ui_interface_string_t *ui_g_Static_Star_Text_4;
extern ui_interface_string_t *ui_g_Static_Star_Text_5;
extern ui_interface_string_t *ui_g_Static_Speed_Text;
extern ui_interface_string_t *ui_g_Static_Stair_Text;
extern ui_interface_string_t *ui_g_Static_Cus_Text;
extern ui_interface_string_t *ui_g_Static_RE_Text;
extern ui_interface_string_t *ui_g_Static_Joy_Text;
extern ui_interface_string_t *ui_g_Static_Pitch1_Text;
extern ui_interface_string_t *ui_g_Static_Pitch2_Text;
extern ui_interface_string_t *ui_g_Static_Leg_Left_Text;

void ui_init_g_Static();
void ui_update_g_Static();
void ui_remove_g_Static();


#endif // UI_g_H
