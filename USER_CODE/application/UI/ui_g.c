//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_Dynamic_0;

ui_interface_number_t *ui_g_Dynamic_Leg_Left_Angle = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[0]);
ui_interface_number_t *ui_g_Dynamic_Leg_Right_Angle = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[1]);
ui_interface_number_t *ui_g_Dynamic_Vy_Speed = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[2]);
ui_interface_number_t *ui_g_Dynamic_Speed = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[3]);
ui_interface_number_t *ui_g_Dynamic_Stair_State_Text = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[4]);
ui_interface_number_t *ui_g_Dynamic_Cus_State_Text = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[5]);
ui_interface_number_t *ui_g_Dynamic_RE_State_Text = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[6]);

void _ui_init_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].figure_name[0] = 0;
        ui_g_Dynamic_0.data[i].figure_name[1] = 0;
        ui_g_Dynamic_0.data[i].figure_name[2] = i + 0;
        ui_g_Dynamic_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 0;
    }

    ui_g_Dynamic_Leg_Left_Angle->figure_type = 6;
    ui_g_Dynamic_Leg_Left_Angle->operate_type = 1;
    ui_g_Dynamic_Leg_Left_Angle->layer = 0;
    ui_g_Dynamic_Leg_Left_Angle->color = 6;
    ui_g_Dynamic_Leg_Left_Angle->start_x = 1759;
    ui_g_Dynamic_Leg_Left_Angle->start_y = 762;
    ui_g_Dynamic_Leg_Left_Angle->width = 2;
    ui_g_Dynamic_Leg_Left_Angle->font_size = 22;
    ui_g_Dynamic_Leg_Left_Angle->number = 0;

    ui_g_Dynamic_Leg_Right_Angle->figure_type = 6;
    ui_g_Dynamic_Leg_Right_Angle->operate_type = 1;
    ui_g_Dynamic_Leg_Right_Angle->layer = 0;
    ui_g_Dynamic_Leg_Right_Angle->color = 6;
    ui_g_Dynamic_Leg_Right_Angle->start_x = 1759;
    ui_g_Dynamic_Leg_Right_Angle->start_y = 722;
    ui_g_Dynamic_Leg_Right_Angle->width = 2;
    ui_g_Dynamic_Leg_Right_Angle->font_size = 22;
    ui_g_Dynamic_Leg_Right_Angle->number = 0;

    ui_g_Dynamic_Vy_Speed->figure_type = 6;
    ui_g_Dynamic_Vy_Speed->operate_type = 1;
    ui_g_Dynamic_Vy_Speed->layer = 0;
    ui_g_Dynamic_Vy_Speed->color = 6;
    ui_g_Dynamic_Vy_Speed->start_x = 1758;
    ui_g_Dynamic_Vy_Speed->start_y = 599;
    ui_g_Dynamic_Vy_Speed->width = 2;
    ui_g_Dynamic_Vy_Speed->font_size = 22;
    ui_g_Dynamic_Vy_Speed->number = 0;

    ui_g_Dynamic_Speed->figure_type = 6;
    ui_g_Dynamic_Speed->operate_type = 1;
    ui_g_Dynamic_Speed->layer = 0;
    ui_g_Dynamic_Speed->color = 6;
    ui_g_Dynamic_Speed->start_x = 264;
    ui_g_Dynamic_Speed->start_y = 891;
    ui_g_Dynamic_Speed->width = 2;
    ui_g_Dynamic_Speed->font_size = 22;
    ui_g_Dynamic_Speed->number = 1;

    ui_g_Dynamic_Stair_State_Text->figure_type = 6;
    ui_g_Dynamic_Stair_State_Text->operate_type = 1;
    ui_g_Dynamic_Stair_State_Text->layer = 0;
    ui_g_Dynamic_Stair_State_Text->color = 5;
    ui_g_Dynamic_Stair_State_Text->start_x = 264;
    ui_g_Dynamic_Stair_State_Text->start_y = 848;
    ui_g_Dynamic_Stair_State_Text->width = 2;
    ui_g_Dynamic_Stair_State_Text->font_size = 22;
    ui_g_Dynamic_Stair_State_Text->number = 0;

    ui_g_Dynamic_Cus_State_Text->figure_type = 6;
    ui_g_Dynamic_Cus_State_Text->operate_type = 1;
    ui_g_Dynamic_Cus_State_Text->layer = 0;
    ui_g_Dynamic_Cus_State_Text->color = 2;
    ui_g_Dynamic_Cus_State_Text->start_x = 264;
    ui_g_Dynamic_Cus_State_Text->start_y = 808;
    ui_g_Dynamic_Cus_State_Text->width = 2;
    ui_g_Dynamic_Cus_State_Text->font_size = 22;
    ui_g_Dynamic_Cus_State_Text->number = 1;

    ui_g_Dynamic_RE_State_Text->figure_type = 6;
    ui_g_Dynamic_RE_State_Text->operate_type = 1;
    ui_g_Dynamic_RE_State_Text->layer = 0;
    ui_g_Dynamic_RE_State_Text->color = 5;
    ui_g_Dynamic_RE_State_Text->start_x = 264;
    ui_g_Dynamic_RE_State_Text->start_y = 762;
    ui_g_Dynamic_RE_State_Text->width = 2;
    ui_g_Dynamic_RE_State_Text->font_size = 22;
    ui_g_Dynamic_RE_State_Text->number = 0;


    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_update_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_remove_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}
ui_5_frame_t ui_g_Dynamic_1;

ui_interface_number_t *ui_g_Dynamic_Joy_State_Text = (ui_interface_number_t*)&(ui_g_Dynamic_1.data[0]);
ui_interface_number_t *ui_g_Dynamic_Pitch1_Angle = (ui_interface_number_t*)&(ui_g_Dynamic_1.data[1]);
ui_interface_number_t *ui_g_Dynamic_Pitch2_Angle = (ui_interface_number_t*)&(ui_g_Dynamic_1.data[2]);

void _ui_init_g_Dynamic_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_Dynamic_1.data[i].figure_name[0] = 0;
        ui_g_Dynamic_1.data[i].figure_name[1] = 0;
        ui_g_Dynamic_1.data[i].figure_name[2] = i + 7;
        ui_g_Dynamic_1.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 0;
    }

    ui_g_Dynamic_Joy_State_Text->figure_type = 6;
    ui_g_Dynamic_Joy_State_Text->operate_type = 1;
    ui_g_Dynamic_Joy_State_Text->layer = 0;
    ui_g_Dynamic_Joy_State_Text->color = 5;
    ui_g_Dynamic_Joy_State_Text->start_x = 264;
    ui_g_Dynamic_Joy_State_Text->start_y = 722;
    ui_g_Dynamic_Joy_State_Text->width = 2;
    ui_g_Dynamic_Joy_State_Text->font_size = 22;
    ui_g_Dynamic_Joy_State_Text->number = 0;

    ui_g_Dynamic_Pitch1_Angle->figure_type = 6;
    ui_g_Dynamic_Pitch1_Angle->operate_type = 1;
    ui_g_Dynamic_Pitch1_Angle->layer = 0;
    ui_g_Dynamic_Pitch1_Angle->color = 6;
    ui_g_Dynamic_Pitch1_Angle->start_x = 1758;
    ui_g_Dynamic_Pitch1_Angle->start_y = 679;
    ui_g_Dynamic_Pitch1_Angle->width = 2;
    ui_g_Dynamic_Pitch1_Angle->font_size = 22;
    ui_g_Dynamic_Pitch1_Angle->number = 0;

    ui_g_Dynamic_Pitch2_Angle->figure_type = 6;
    ui_g_Dynamic_Pitch2_Angle->operate_type = 1;
    ui_g_Dynamic_Pitch2_Angle->layer = 0;
    ui_g_Dynamic_Pitch2_Angle->color = 6;
    ui_g_Dynamic_Pitch2_Angle->start_x = 1760;
    ui_g_Dynamic_Pitch2_Angle->start_y = 639;
    ui_g_Dynamic_Pitch2_Angle->width = 2;
    ui_g_Dynamic_Pitch2_Angle->font_size = 22;
    ui_g_Dynamic_Pitch2_Angle->number = 0;


    ui_proc_5_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_update_g_Dynamic_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_remove_g_Dynamic_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}


void ui_init_g_Dynamic() {
    _ui_init_g_Dynamic_0();
    _ui_init_g_Dynamic_1();
}

void ui_update_g_Dynamic() {
    _ui_update_g_Dynamic_0();
    _ui_update_g_Dynamic_1();
}

void ui_remove_g_Dynamic() {
    _ui_remove_g_Dynamic_0();
    _ui_remove_g_Dynamic_1();
}

ui_5_frame_t ui_g_Static_0;

ui_interface_line_t *ui_g_Static_Star_2 = (ui_interface_line_t*)&(ui_g_Static_0.data[0]);
ui_interface_line_t *ui_g_Static_Star_3 = (ui_interface_line_t*)&(ui_g_Static_0.data[1]);
ui_interface_line_t *ui_g_Static_Star_4 = (ui_interface_line_t*)&(ui_g_Static_0.data[2]);
ui_interface_line_t *ui_g_Static_Star_5 = (ui_interface_line_t*)&(ui_g_Static_0.data[3]);
ui_interface_line_t *ui_g_Static_Star_6 = (ui_interface_line_t*)&(ui_g_Static_0.data[4]);

void _ui_init_g_Static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Static_0.data[i].figure_name[0] = 0;
        ui_g_Static_0.data[i].figure_name[1] = 1;
        ui_g_Static_0.data[i].figure_name[2] = i + 0;
        ui_g_Static_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_Static_0.data[i].operate_type = 0;
    }

    ui_g_Static_Star_2->figure_type = 0;
    ui_g_Static_Star_2->operate_type = 1;
    ui_g_Static_Star_2->layer = 0;
    ui_g_Static_Star_2->color = 8;
    ui_g_Static_Star_2->start_x = 917;
    ui_g_Static_Star_2->start_y = 522;
    ui_g_Static_Star_2->width = 1;
    ui_g_Static_Star_2->end_x = 1017;
    ui_g_Static_Star_2->end_y = 522;

    ui_g_Static_Star_3->figure_type = 0;
    ui_g_Static_Star_3->operate_type = 1;
    ui_g_Static_Star_3->layer = 0;
    ui_g_Static_Star_3->color = 8;
    ui_g_Static_Star_3->start_x = 867;
    ui_g_Static_Star_3->start_y = 482;
    ui_g_Static_Star_3->width = 1;
    ui_g_Static_Star_3->end_x = 1068;
    ui_g_Static_Star_3->end_y = 482;

    ui_g_Static_Star_4->figure_type = 0;
    ui_g_Static_Star_4->operate_type = 1;
    ui_g_Static_Star_4->layer = 0;
    ui_g_Static_Star_4->color = 8;
    ui_g_Static_Star_4->start_x = 817;
    ui_g_Static_Star_4->start_y = 442;
    ui_g_Static_Star_4->width = 1;
    ui_g_Static_Star_4->end_x = 1116;
    ui_g_Static_Star_4->end_y = 442;

    ui_g_Static_Star_5->figure_type = 0;
    ui_g_Static_Star_5->operate_type = 1;
    ui_g_Static_Star_5->layer = 0;
    ui_g_Static_Star_5->color = 8;
    ui_g_Static_Star_5->start_x = 767;
    ui_g_Static_Star_5->start_y = 402;
    ui_g_Static_Star_5->width = 1;
    ui_g_Static_Star_5->end_x = 1168;
    ui_g_Static_Star_5->end_y = 402;

    ui_g_Static_Star_6->figure_type = 0;
    ui_g_Static_Star_6->operate_type = 1;
    ui_g_Static_Star_6->layer = 0;
    ui_g_Static_Star_6->color = 0;
    ui_g_Static_Star_6->start_x = 717;
    ui_g_Static_Star_6->start_y = 362;
    ui_g_Static_Star_6->width = 1;
    ui_g_Static_Star_6->end_x = 1214;
    ui_g_Static_Star_6->end_y = 362;


    ui_proc_5_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_update_g_Static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Static_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_remove_g_Static_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Static_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

ui_string_frame_t ui_g_Static_1;
ui_interface_string_t* ui_g_Static_Leg_Right_Text = &(ui_g_Static_1.option);

void _ui_init_g_Static_1() {
    ui_g_Static_1.option.figure_name[0] = 0;
    ui_g_Static_1.option.figure_name[1] = 1;
    ui_g_Static_1.option.figure_name[2] = 5;
    ui_g_Static_1.option.operate_type = 1;

    ui_g_Static_Leg_Right_Text->figure_type = 7;
    ui_g_Static_Leg_Right_Text->operate_type = 1;
    ui_g_Static_Leg_Right_Text->layer = 0;
    ui_g_Static_Leg_Right_Text->color = 3;
    ui_g_Static_Leg_Right_Text->start_x = 1523;
    ui_g_Static_Leg_Right_Text->start_y = 717;
    ui_g_Static_Leg_Right_Text->width = 2;
    ui_g_Static_Leg_Right_Text->font_size = 22;
    ui_g_Static_Leg_Right_Text->str_length = 10;
    strcpy(ui_g_Static_Leg_Right_Text->string, "LEG_RIGHT:");


    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_update_g_Static_1() {
    ui_g_Static_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_remove_g_Static_1() {
    ui_g_Static_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}
ui_string_frame_t ui_g_Static_2;
ui_interface_string_t* ui_g_Static_Vy_Text = &(ui_g_Static_2.option);

void _ui_init_g_Static_2() {
    ui_g_Static_2.option.figure_name[0] = 0;
    ui_g_Static_2.option.figure_name[1] = 1;
    ui_g_Static_2.option.figure_name[2] = 6;
    ui_g_Static_2.option.operate_type = 1;

    ui_g_Static_Vy_Text->figure_type = 7;
    ui_g_Static_Vy_Text->operate_type = 1;
    ui_g_Static_Vy_Text->layer = 0;
    ui_g_Static_Vy_Text->color = 0;
    ui_g_Static_Vy_Text->start_x = 1676;
    ui_g_Static_Vy_Text->start_y = 598;
    ui_g_Static_Vy_Text->width = 2;
    ui_g_Static_Vy_Text->font_size = 22;
    ui_g_Static_Vy_Text->str_length = 3;
    strcpy(ui_g_Static_Vy_Text->string, "Vy:");


    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_update_g_Static_2() {
    ui_g_Static_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_remove_g_Static_2() {
    ui_g_Static_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}
ui_string_frame_t ui_g_Static_3;
ui_interface_string_t* ui_g_Static_WUST_RM = &(ui_g_Static_3.option);

void _ui_init_g_Static_3() {
    ui_g_Static_3.option.figure_name[0] = 0;
    ui_g_Static_3.option.figure_name[1] = 1;
    ui_g_Static_3.option.figure_name[2] = 7;
    ui_g_Static_3.option.operate_type = 1;

    ui_g_Static_WUST_RM->figure_type = 7;
    ui_g_Static_WUST_RM->operate_type = 1;
    ui_g_Static_WUST_RM->layer = 0;
    ui_g_Static_WUST_RM->color = 8;
    ui_g_Static_WUST_RM->start_x = 1720;
    ui_g_Static_WUST_RM->start_y = 317;
    ui_g_Static_WUST_RM->width = 2;
    ui_g_Static_WUST_RM->font_size = 20;
    ui_g_Static_WUST_RM->str_length = 7;
    strcpy(ui_g_Static_WUST_RM->string, "WUST_RM");


    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_update_g_Static_3() {
    ui_g_Static_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_remove_g_Static_3() {
    ui_g_Static_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}
ui_string_frame_t ui_g_Static_4;
ui_interface_string_t* ui_g_Static_Star_Text_1 = &(ui_g_Static_4.option);

void _ui_init_g_Static_4() {
    ui_g_Static_4.option.figure_name[0] = 0;
    ui_g_Static_4.option.figure_name[1] = 1;
    ui_g_Static_4.option.figure_name[2] = 8;
    ui_g_Static_4.option.operate_type = 1;

    ui_g_Static_Star_Text_1->figure_type = 7;
    ui_g_Static_Star_Text_1->operate_type = 1;
    ui_g_Static_Star_Text_1->layer = 0;
    ui_g_Static_Star_Text_1->color = 8;
    ui_g_Static_Star_Text_1->start_x = 891;
    ui_g_Static_Star_Text_1->start_y = 534;
    ui_g_Static_Star_Text_1->width = 2;
    ui_g_Static_Star_Text_1->font_size = 15;
    ui_g_Static_Star_Text_1->str_length = 1;
    strcpy(ui_g_Static_Star_Text_1->string, "1");


    ui_proc_string_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}

void _ui_update_g_Static_4() {
    ui_g_Static_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}

void _ui_remove_g_Static_4() {
    ui_g_Static_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}
ui_string_frame_t ui_g_Static_5;
ui_interface_string_t* ui_g_Static_Star_Text_2 = &(ui_g_Static_5.option);

void _ui_init_g_Static_5() {
    ui_g_Static_5.option.figure_name[0] = 0;
    ui_g_Static_5.option.figure_name[1] = 1;
    ui_g_Static_5.option.figure_name[2] = 9;
    ui_g_Static_5.option.operate_type = 1;

    ui_g_Static_Star_Text_2->figure_type = 7;
    ui_g_Static_Star_Text_2->operate_type = 1;
    ui_g_Static_Star_Text_2->layer = 0;
    ui_g_Static_Star_Text_2->color = 8;
    ui_g_Static_Star_Text_2->start_x = 840;
    ui_g_Static_Star_Text_2->start_y = 493;
    ui_g_Static_Star_Text_2->width = 2;
    ui_g_Static_Star_Text_2->font_size = 15;
    ui_g_Static_Star_Text_2->str_length = 1;
    strcpy(ui_g_Static_Star_Text_2->string, "2");


    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}

void _ui_update_g_Static_5() {
    ui_g_Static_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}

void _ui_remove_g_Static_5() {
    ui_g_Static_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}
ui_string_frame_t ui_g_Static_6;
ui_interface_string_t* ui_g_Static_Star_Text_3 = &(ui_g_Static_6.option);

void _ui_init_g_Static_6() {
    ui_g_Static_6.option.figure_name[0] = 0;
    ui_g_Static_6.option.figure_name[1] = 1;
    ui_g_Static_6.option.figure_name[2] = 10;
    ui_g_Static_6.option.operate_type = 1;

    ui_g_Static_Star_Text_3->figure_type = 7;
    ui_g_Static_Star_Text_3->operate_type = 1;
    ui_g_Static_Star_Text_3->layer = 0;
    ui_g_Static_Star_Text_3->color = 8;
    ui_g_Static_Star_Text_3->start_x = 789;
    ui_g_Static_Star_Text_3->start_y = 454;
    ui_g_Static_Star_Text_3->width = 2;
    ui_g_Static_Star_Text_3->font_size = 15;
    ui_g_Static_Star_Text_3->str_length = 1;
    strcpy(ui_g_Static_Star_Text_3->string, "3");


    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}

void _ui_update_g_Static_6() {
    ui_g_Static_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}

void _ui_remove_g_Static_6() {
    ui_g_Static_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}
ui_string_frame_t ui_g_Static_7;
ui_interface_string_t* ui_g_Static_Star_Text_4 = &(ui_g_Static_7.option);

void _ui_init_g_Static_7() {
    ui_g_Static_7.option.figure_name[0] = 0;
    ui_g_Static_7.option.figure_name[1] = 1;
    ui_g_Static_7.option.figure_name[2] = 11;
    ui_g_Static_7.option.operate_type = 1;

    ui_g_Static_Star_Text_4->figure_type = 7;
    ui_g_Static_Star_Text_4->operate_type = 1;
    ui_g_Static_Star_Text_4->layer = 0;
    ui_g_Static_Star_Text_4->color = 8;
    ui_g_Static_Star_Text_4->start_x = 738;
    ui_g_Static_Star_Text_4->start_y = 412;
    ui_g_Static_Star_Text_4->width = 2;
    ui_g_Static_Star_Text_4->font_size = 15;
    ui_g_Static_Star_Text_4->str_length = 1;
    strcpy(ui_g_Static_Star_Text_4->string, "4");


    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}

void _ui_update_g_Static_7() {
    ui_g_Static_7.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}

void _ui_remove_g_Static_7() {
    ui_g_Static_7.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}
ui_string_frame_t ui_g_Static_8;
ui_interface_string_t* ui_g_Static_Star_Text_5 = &(ui_g_Static_8.option);

void _ui_init_g_Static_8() {
    ui_g_Static_8.option.figure_name[0] = 0;
    ui_g_Static_8.option.figure_name[1] = 1;
    ui_g_Static_8.option.figure_name[2] = 12;
    ui_g_Static_8.option.operate_type = 1;

    ui_g_Static_Star_Text_5->figure_type = 7;
    ui_g_Static_Star_Text_5->operate_type = 1;
    ui_g_Static_Star_Text_5->layer = 0;
    ui_g_Static_Star_Text_5->color = 8;
    ui_g_Static_Star_Text_5->start_x = 689;
    ui_g_Static_Star_Text_5->start_y = 374;
    ui_g_Static_Star_Text_5->width = 2;
    ui_g_Static_Star_Text_5->font_size = 15;
    ui_g_Static_Star_Text_5->str_length = 1;
    strcpy(ui_g_Static_Star_Text_5->string, "5");


    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}

void _ui_update_g_Static_8() {
    ui_g_Static_8.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}

void _ui_remove_g_Static_8() {
    ui_g_Static_8.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}
ui_string_frame_t ui_g_Static_9;
ui_interface_string_t* ui_g_Static_Speed_Text = &(ui_g_Static_9.option);

void _ui_init_g_Static_9() {
    ui_g_Static_9.option.figure_name[0] = 0;
    ui_g_Static_9.option.figure_name[1] = 1;
    ui_g_Static_9.option.figure_name[2] = 13;
    ui_g_Static_9.option.operate_type = 1;

    ui_g_Static_Speed_Text->figure_type = 7;
    ui_g_Static_Speed_Text->operate_type = 1;
    ui_g_Static_Speed_Text->layer = 0;
    ui_g_Static_Speed_Text->color = 8;
    ui_g_Static_Speed_Text->start_x = 102;
    ui_g_Static_Speed_Text->start_y = 889;
    ui_g_Static_Speed_Text->width = 2;
    ui_g_Static_Speed_Text->font_size = 22;
    ui_g_Static_Speed_Text->str_length = 7;
    strcpy(ui_g_Static_Speed_Text->string, "Spd_Lv:");


    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}

void _ui_update_g_Static_9() {
    ui_g_Static_9.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}

void _ui_remove_g_Static_9() {
    ui_g_Static_9.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}
ui_string_frame_t ui_g_Static_10;
ui_interface_string_t* ui_g_Static_Stair_Text = &(ui_g_Static_10.option);

void _ui_init_g_Static_10() {
    ui_g_Static_10.option.figure_name[0] = 0;
    ui_g_Static_10.option.figure_name[1] = 1;
    ui_g_Static_10.option.figure_name[2] = 14;
    ui_g_Static_10.option.operate_type = 1;

    ui_g_Static_Stair_Text->figure_type = 7;
    ui_g_Static_Stair_Text->operate_type = 1;
    ui_g_Static_Stair_Text->layer = 0;
    ui_g_Static_Stair_Text->color = 8;
    ui_g_Static_Stair_Text->start_x = 13;
    ui_g_Static_Stair_Text->start_y = 848;
    ui_g_Static_Stair_Text->width = 2;
    ui_g_Static_Stair_Text->font_size = 22;
    ui_g_Static_Stair_Text->str_length = 11;
    strcpy(ui_g_Static_Stair_Text->string, "Stair_Auto:");


    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}

void _ui_update_g_Static_10() {
    ui_g_Static_10.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}

void _ui_remove_g_Static_10() {
    ui_g_Static_10.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}
ui_string_frame_t ui_g_Static_11;
ui_interface_string_t* ui_g_Static_Cus_Text = &(ui_g_Static_11.option);

void _ui_init_g_Static_11() {
    ui_g_Static_11.option.figure_name[0] = 0;
    ui_g_Static_11.option.figure_name[1] = 1;
    ui_g_Static_11.option.figure_name[2] = 15;
    ui_g_Static_11.option.operate_type = 1;

    ui_g_Static_Cus_Text->figure_type = 7;
    ui_g_Static_Cus_Text->operate_type = 1;
    ui_g_Static_Cus_Text->layer = 0;
    ui_g_Static_Cus_Text->color = 8;
    ui_g_Static_Cus_Text->start_x = 58;
    ui_g_Static_Cus_Text->start_y = 808;
    ui_g_Static_Cus_Text->width = 2;
    ui_g_Static_Cus_Text->font_size = 22;
    ui_g_Static_Cus_Text->str_length = 9;
    strcpy(ui_g_Static_Cus_Text->string, "Cus_Ctrl:");


    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}

void _ui_update_g_Static_11() {
    ui_g_Static_11.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}

void _ui_remove_g_Static_11() {
    ui_g_Static_11.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}
ui_string_frame_t ui_g_Static_12;
ui_interface_string_t* ui_g_Static_RE_Text = &(ui_g_Static_12.option);

void _ui_init_g_Static_12() {
    ui_g_Static_12.option.figure_name[0] = 0;
    ui_g_Static_12.option.figure_name[1] = 1;
    ui_g_Static_12.option.figure_name[2] = 16;
    ui_g_Static_12.option.operate_type = 1;

    ui_g_Static_RE_Text->figure_type = 7;
    ui_g_Static_RE_Text->operate_type = 1;
    ui_g_Static_RE_Text->layer = 0;
    ui_g_Static_RE_Text->color = 8;
    ui_g_Static_RE_Text->start_x = 78;
    ui_g_Static_RE_Text->start_y = 763;
    ui_g_Static_RE_Text->width = 2;
    ui_g_Static_RE_Text->font_size = 22;
    ui_g_Static_RE_Text->str_length = 8;
    strcpy(ui_g_Static_RE_Text->string, "RE_MODE:");


    ui_proc_string_frame(&ui_g_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_12, sizeof(ui_g_Static_12));
}

void _ui_update_g_Static_12() {
    ui_g_Static_12.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_12, sizeof(ui_g_Static_12));
}

void _ui_remove_g_Static_12() {
    ui_g_Static_12.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_12, sizeof(ui_g_Static_12));
}
ui_string_frame_t ui_g_Static_13;
ui_interface_string_t* ui_g_Static_Joy_Text = &(ui_g_Static_13.option);

void _ui_init_g_Static_13() {
    ui_g_Static_13.option.figure_name[0] = 0;
    ui_g_Static_13.option.figure_name[1] = 1;
    ui_g_Static_13.option.figure_name[2] = 17;
    ui_g_Static_13.option.operate_type = 1;

    ui_g_Static_Joy_Text->figure_type = 7;
    ui_g_Static_Joy_Text->operate_type = 1;
    ui_g_Static_Joy_Text->layer = 0;
    ui_g_Static_Joy_Text->color = 8;
    ui_g_Static_Joy_Text->start_x = 56;
    ui_g_Static_Joy_Text->start_y = 723;
    ui_g_Static_Joy_Text->width = 2;
    ui_g_Static_Joy_Text->font_size = 22;
    ui_g_Static_Joy_Text->str_length = 9;
    strcpy(ui_g_Static_Joy_Text->string, "JOY_MODE:");


    ui_proc_string_frame(&ui_g_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_13, sizeof(ui_g_Static_13));
}

void _ui_update_g_Static_13() {
    ui_g_Static_13.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_13, sizeof(ui_g_Static_13));
}

void _ui_remove_g_Static_13() {
    ui_g_Static_13.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_13, sizeof(ui_g_Static_13));
}
ui_string_frame_t ui_g_Static_14;
ui_interface_string_t* ui_g_Static_Pitch1_Text = &(ui_g_Static_14.option);

void _ui_init_g_Static_14() {
    ui_g_Static_14.option.figure_name[0] = 0;
    ui_g_Static_14.option.figure_name[1] = 1;
    ui_g_Static_14.option.figure_name[2] = 18;
    ui_g_Static_14.option.operate_type = 1;

    ui_g_Static_Pitch1_Text->figure_type = 7;
    ui_g_Static_Pitch1_Text->operate_type = 1;
    ui_g_Static_Pitch1_Text->layer = 0;
    ui_g_Static_Pitch1_Text->color = 1;
    ui_g_Static_Pitch1_Text->start_x = 1589;
    ui_g_Static_Pitch1_Text->start_y = 679;
    ui_g_Static_Pitch1_Text->width = 2;
    ui_g_Static_Pitch1_Text->font_size = 22;
    ui_g_Static_Pitch1_Text->str_length = 7;
    strcpy(ui_g_Static_Pitch1_Text->string, "Pitch1:");


    ui_proc_string_frame(&ui_g_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_14, sizeof(ui_g_Static_14));
}

void _ui_update_g_Static_14() {
    ui_g_Static_14.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_14, sizeof(ui_g_Static_14));
}

void _ui_remove_g_Static_14() {
    ui_g_Static_14.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_14, sizeof(ui_g_Static_14));
}
ui_string_frame_t ui_g_Static_15;
ui_interface_string_t* ui_g_Static_Pitch2_Text = &(ui_g_Static_15.option);

void _ui_init_g_Static_15() {
    ui_g_Static_15.option.figure_name[0] = 0;
    ui_g_Static_15.option.figure_name[1] = 1;
    ui_g_Static_15.option.figure_name[2] = 19;
    ui_g_Static_15.option.operate_type = 1;

    ui_g_Static_Pitch2_Text->figure_type = 7;
    ui_g_Static_Pitch2_Text->operate_type = 1;
    ui_g_Static_Pitch2_Text->layer = 0;
    ui_g_Static_Pitch2_Text->color = 1;
    ui_g_Static_Pitch2_Text->start_x = 1589;
    ui_g_Static_Pitch2_Text->start_y = 636;
    ui_g_Static_Pitch2_Text->width = 2;
    ui_g_Static_Pitch2_Text->font_size = 22;
    ui_g_Static_Pitch2_Text->str_length = 7;
    strcpy(ui_g_Static_Pitch2_Text->string, "Pitch2:");


    ui_proc_string_frame(&ui_g_Static_15);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_15, sizeof(ui_g_Static_15));
}

void _ui_update_g_Static_15() {
    ui_g_Static_15.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_15);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_15, sizeof(ui_g_Static_15));
}

void _ui_remove_g_Static_15() {
    ui_g_Static_15.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_15);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_15, sizeof(ui_g_Static_15));
}
ui_string_frame_t ui_g_Static_16;
ui_interface_string_t* ui_g_Static_Leg_Left_Text = &(ui_g_Static_16.option);

void _ui_init_g_Static_16() {
    ui_g_Static_16.option.figure_name[0] = 0;
    ui_g_Static_16.option.figure_name[1] = 1;
    ui_g_Static_16.option.figure_name[2] = 20;
    ui_g_Static_16.option.operate_type = 1;

    ui_g_Static_Leg_Left_Text->figure_type = 7;
    ui_g_Static_Leg_Left_Text->operate_type = 1;
    ui_g_Static_Leg_Left_Text->layer = 0;
    ui_g_Static_Leg_Left_Text->color = 3;
    ui_g_Static_Leg_Left_Text->start_x = 1543;
    ui_g_Static_Leg_Left_Text->start_y = 757;
    ui_g_Static_Leg_Left_Text->width = 2;
    ui_g_Static_Leg_Left_Text->font_size = 22;
    ui_g_Static_Leg_Left_Text->str_length = 9;
    strcpy(ui_g_Static_Leg_Left_Text->string, "LEG_LEFT:");


    ui_proc_string_frame(&ui_g_Static_16);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_16, sizeof(ui_g_Static_16));
}

void _ui_update_g_Static_16() {
    ui_g_Static_16.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_16);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_16, sizeof(ui_g_Static_16));
}

void _ui_remove_g_Static_16() {
    ui_g_Static_16.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_16);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_16, sizeof(ui_g_Static_16));
}

void ui_init_g_Static() {
    _ui_init_g_Static_0();
    _ui_init_g_Static_1();
    _ui_init_g_Static_2();
    _ui_init_g_Static_3();
    _ui_init_g_Static_4();
    _ui_init_g_Static_5();
    _ui_init_g_Static_6();
    _ui_init_g_Static_7();
    _ui_init_g_Static_8();
    _ui_init_g_Static_9();
    _ui_init_g_Static_10();
    _ui_init_g_Static_11();
    _ui_init_g_Static_12();
    _ui_init_g_Static_13();
    _ui_init_g_Static_14();
    _ui_init_g_Static_15();
    _ui_init_g_Static_16();
}

void ui_update_g_Static() {
    _ui_update_g_Static_0();
    _ui_update_g_Static_1();
    _ui_update_g_Static_2();
    _ui_update_g_Static_3();
    _ui_update_g_Static_4();
    _ui_update_g_Static_5();
    _ui_update_g_Static_6();
    _ui_update_g_Static_7();
    _ui_update_g_Static_8();
    _ui_update_g_Static_9();
    _ui_update_g_Static_10();
    _ui_update_g_Static_11();
    _ui_update_g_Static_12();
    _ui_update_g_Static_13();
    _ui_update_g_Static_14();
    _ui_update_g_Static_15();
    _ui_update_g_Static_16();
}

void ui_remove_g_Static() {
    _ui_remove_g_Static_0();
    _ui_remove_g_Static_1();
    _ui_remove_g_Static_2();
    _ui_remove_g_Static_3();
    _ui_remove_g_Static_4();
    _ui_remove_g_Static_5();
    _ui_remove_g_Static_6();
    _ui_remove_g_Static_7();
    _ui_remove_g_Static_8();
    _ui_remove_g_Static_9();
    _ui_remove_g_Static_10();
    _ui_remove_g_Static_11();
    _ui_remove_g_Static_12();
    _ui_remove_g_Static_13();
    _ui_remove_g_Static_14();
    _ui_remove_g_Static_15();
    _ui_remove_g_Static_16();
}

