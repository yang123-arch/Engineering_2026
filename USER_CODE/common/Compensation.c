//======================================================================
//                      compensation[补偿]
//               使机械臂达到补偿的效果
//======================================================================

#include "Compensation.h"
#include "Gimbal.h"
#include "RoboControl.h"

Homogeneous_StructTypeDef T[Joint_NUM]; //连杆i相对于i-1连杆的齐次变换矩阵
Matrix_StructTypeDef R_T[Joint_NUM]; //旋转矩阵的转置
Vector_StructTypeDef acc[Joint_NUM]; //连杆i自身坐标系下的等效重力加速度
Vector_StructTypeDef p_i1_i[Joint_NUM]; //连杆i+1原点在连杆i坐标系下的位置
Vector_StructTypeDef a0 = {0.0f, 0.0f, -GRAVITY}; //基座等效加速度初始值
Matrix_StructTypeDef R_joint;
Homogeneous_StructTypeDef T_joint;
Vector_StructTypeDef f_tcp_on_link7;
Vector_StructTypeDef n_tcp_on_link7;
Matrix_StructTypeDef R_tcp_T;
Vector_StructTypeDef a_tcp;
Vector_StructTypeDef F_g_tcp;
Vector_StructTypeDef M_g_tcp;
Vector_StructTypeDef n_g_tcp_on_link7;
Vector_StructTypeDef M_f_tcp_on_link7;
Vector_StructTypeDef f[Joint_NUM + 1]; //连杆i受到连杆i-1的作用力
Vector_StructTypeDef n[Joint_NUM + 1]; //连杆i受到连杆i-1的作用力矩
Vector_StructTypeDef F_g;
Vector_StructTypeDef f_next;
Matrix_StructTypeDef R_i1_i;
Vector_StructTypeDef n_next;
Vector_StructTypeDef M_next;
Vector_StructTypeDef M_g;

/**
 * @brief  参数初始函数
 * @param  连杆参数数组
 * @retval 无
 */
void link_init(Link_StructTypeDef link[Joint_NUM + 1]) {
  link[0].m = 1.31122096f;
  link[0].p_ci.x = -0.02990190f;
  link[0].p_ci.y = 0.01420106f;
  link[0].p_ci.z = 0.04887729f;
  rpy_to_rot(0.0f, 0.0f, 0.0f, &link[0].T_fixed.R);
  link[0].T_fixed.p.x = 0.0f;
  link[0].T_fixed.p.y = 0.0f;
  link[0].T_fixed.p.z = 0.228f;
  link[0].axis.x = 0.0f;
  link[0].axis.y = 0.0f;
  link[0].axis.z = 1.0f;
  link[0].Joint_torque = 0.0f;
  link[0].Joint_torque_output = 0.0f;
  link[0].q = 0.0f;
  link[0].q_offset = 0.0f;

  link[1].m = 1.93449388f;
  link[1].p_ci.x = 0.17627996f;
  link[1].p_ci.y = -0.00250778f;
  link[1].p_ci.z = 0.00421522f;
  rpy_to_rot(PI / 2.0f, -0.77059f, PI, &link[1].T_fixed.R);
  link[1].T_fixed.p.x = 0.024226f;
  link[1].T_fixed.p.y = -0.0557f;
  link[1].T_fixed.p.z = 0.073f;
  link[1].axis.x = 0.0f;
  link[1].axis.y = 0.0f;
  link[1].axis.z = 1.0f;
  link[1].Joint_torque = 0.0f;
  link[1].Joint_torque_output = 0.0f;
  link[1].q = 0.0f;
  link[1].q_offset = -20.85f;

  link[2].m = 1.93928154f;
  link[2].p_ci.x = 0.03863854f;
  link[2].p_ci.y = 0.09287914f;
  link[2].p_ci.z = -0.04799719f;
  rpy_to_rot(-PI, 0.0f, 2.371f, &link[2].T_fixed.R);
  link[2].T_fixed.p.x = 0.308f;
  link[2].T_fixed.p.y = 0.0f;
  link[2].T_fixed.p.z = 0.0498f;
  link[2].axis.x = 0.0f;
  link[2].axis.y = 0.0f;
  link[2].axis.z = 1.0f;
  link[2].Joint_torque = 0.0f;
  link[2].Joint_torque_output = 0.0f;
  link[2].q = 0.0f;
  link[2].q_offset = -20.85f;

  link[3].m = 0.42994598f;
  link[3].p_ci.x = -0.00098552f;
  link[3].p_ci.y = -0.00030265f;
  link[3].p_ci.z = 0.02830966f;
  rpy_to_rot(PI / 2.0f, 0.0f, PI / 2.0f, &link[3].T_fixed.R);
  link[3].T_fixed.p.x = 0.104f;
  link[3].T_fixed.p.y = 0.0f;
  link[3].T_fixed.p.z = -0.033f;
  link[3].axis.x = 0.0f;
  link[3].axis.y = 0.0f;
  link[3].axis.z = 1.0f;
  link[3].Joint_torque = 0.0f;
  link[3].Joint_torque_output = 0.0f;
  link[3].q = 0.0f;
  link[3].q_offset = 0.0f;

  link[4].m = 0.51486341f;
  link[4].p_ci.x = 0.07993814f;
  link[4].p_ci.y = 0.00202643f;
  link[4].p_ci.z = -0.00006134f;
  rpy_to_rot(PI / 2.0f, -PI / 2.0f, 0.0f, &link[4].T_fixed.R);
  link[4].T_fixed.p.x = 0.0f;
  link[4].T_fixed.p.y = 0.0f;
  link[4].T_fixed.p.z = 0.08f;
  link[4].axis.x = 0.0f;
  link[4].axis.y = 0.0f;
  link[4].axis.z = 1.0f;
  link[4].Joint_torque = 0.0f;
  link[4].Joint_torque_output = 0.0f;
  link[4].q = 0.0f;
  link[4].q_offset = 0.0f;

  link[5].m = 0.26194751f;
  link[5].p_ci.x = 0.00005491f;
  link[5].p_ci.y = -0.02853983f;
  link[5].p_ci.z = -0.00527210f;
  rpy_to_rot(PI / 2.0f, -PI / 2.0f, 0.0f, &link[5].T_fixed.R);
  link[5].T_fixed.p.x = 0.178f;
  link[5].T_fixed.p.y = 0.0067f;
  link[5].T_fixed.p.z = 0.000625f;
  link[5].axis.x = 0.0f;
  link[5].axis.y = 0.0f;
  link[5].axis.z = 1.0f;
  link[5].Joint_torque = 0.0f;
  link[5].Joint_torque_output = 0.0f;
  link[5].q = 0.0f;
  link[5].q_offset = 180.0f;

  link[6].m = 0.62790769f;
  link[6].p_ci.x = 0.00000849f;
  link[6].p_ci.y = 0.00000000f;
  link[6].p_ci.z = 0.00360988f;
  rpy_to_rot(PI / 2.0f, 0.0f, 0.0f, &link[6].T_fixed.R);
  link[6].T_fixed.p.x = 0.0f;
  link[6].T_fixed.p.y = 0.0535f;
  link[6].T_fixed.p.z = 0.0f;
  link[6].axis.x = 0.0f;
  link[6].axis.y = 0.0f;
  link[6].axis.z = 1.0f;
  link[6].Joint_torque = 0.0f;
  link[6].Joint_torque_output = 0.0f;
  link[6].q = 0.0f;
  link[6].q_offset = 0.0f;

  link[7].m = 0.21538562f;
  link[7].p_ci.x = 0.03659999f;
  link[7].p_ci.y = -0.0000002f;
  link[7].p_ci.z = -0.06238217f;
  rpy_to_rot(PI, 0.0f, PI, &link[7].T_fixed.R);
  link[7].T_fixed.p.x = 0.0366f;
  link[7].T_fixed.p.y = 0.0f;
  link[7].T_fixed.p.z = -0.081603f;
  link[7].axis.x = 0.0f;
  link[7].axis.y = 0.0f;
  link[7].axis.z = 1.0f;
  link[7].Joint_torque = 0.0f;
  link[7].Joint_torque_output = 0.0f;
  link[7].q = 0.0f;
  link[7].q_offset = 0.0f;
}

/**
 * @brief 静态重力矩计算函数
 */
void gravity_calc(Link_StructTypeDef arm[Joint_NUM + 1]) {
  if (RoboControl_Struct.Robo_Enable) {
    arm[0].q = ator(Gimbal_Control_Struct.Yaw.dev->data.Total_Angle - arm[0].q_offset);
    arm[1].q = ator(Gimbal_Control_Struct.Pitch1.dev->data.Total_Angle - arm[1].q_offset);
    arm[2].q = ator(Gimbal_Control_Struct.Pitch2.dev->data.Total_Angle - arm[2].q_offset);
    arm[3].q = ator(Gimbal_Control_Struct.J4.dev->data.Total_Angle - arm[3].q_offset);
    arm[4].q = ator(GIMBAL_J5_LEGACY_ANGLE(Gimbal_Control_Struct.J5.dev->data.Total_Angle) -
                    arm[4].q_offset);
    arm[5].q = ator(Gimbal_Control_Struct.J6.dev->data.Total_Angle - arm[5].q_offset);
    arm[6].q = ator(Gimbal_Control_Struct.J7.dev->data.Total_Angle - arm[6].q_offset);

    for (int i = 0; i < Joint_NUM; i++) {
      rot_z(arm[i].q, &R_joint);
      T_joint.R = R_joint;
      vector_zero(&T_joint.p);

      Homogeneous_mul(&arm[i].T_fixed, &T_joint, &T[i]);
      matrix_transpose(&T[i].R, &R_T[i]);

      if (i == 0) {
        matrix_mul_vector(&R_T[i], &a0, &acc[i]);
      } else {
        matrix_mul_vector(&R_T[i], &acc[i - 1], &acc[i]);
      }
    }

    for (int j = 0; j < Joint_NUM - 1; j++) {
      p_i1_i[j] = T[j + 1].p;
    }

    vector_zero(&f_tcp_on_link7);
    vector_zero(&n_tcp_on_link7);

    matrix_transpose(&arm[7].T_fixed.R, &R_tcp_T);
    matrix_mul_vector(&R_tcp_T, &acc[6], &a_tcp);

    vector_mul(&a_tcp, arm[7].m, &F_g_tcp);
    vector_cross(&arm[7].p_ci, &F_g_tcp, &M_g_tcp);

    matrix_mul_vector(&arm[7].T_fixed.R, &F_g_tcp, &f_tcp_on_link7);
    matrix_mul_vector(&arm[7].T_fixed.R, &M_g_tcp, &n_g_tcp_on_link7);
    vector_cross(&arm[7].T_fixed.p, &f_tcp_on_link7, &M_f_tcp_on_link7);

    vector_add(&n_g_tcp_on_link7, &M_f_tcp_on_link7, &n_tcp_on_link7);

    vector_zero(&f[Joint_NUM]);
    vector_zero(&n[Joint_NUM]);

    for (int k = Joint_NUM - 1; k >= 0; k--) {
      vector_mul(&acc[k], arm[k].m, &F_g);

      vector_zero(&f_next);
      matrix_identity(&R_i1_i);
      vector_zero(&n_next);
      vector_zero(&M_next);

      if (k < Joint_NUM - 1) {
        R_i1_i = T[k + 1].R;
        matrix_mul_vector(&R_i1_i, &f[k + 1], &f_next);
        matrix_mul_vector(&R_i1_i, &n[k + 1], &n_next);
        vector_cross(&p_i1_i[k], &f_next, &M_next);
      } else {
        f_next = f_tcp_on_link7;
        n_next = n_tcp_on_link7;
        vector_zero(&M_next);
      }

      vector_add(&f_next, &F_g, &f[k]);
      vector_cross(&arm[k].p_ci, &F_g, &M_g);

      // n[k] = vector_add(vector_add(n_next, M_g), M_next);
      Vector_StructTypeDef temp_n;
      vector_add(&n_next, &M_g, &temp_n);
      vector_add(&temp_n, &M_next, &n[k]);

      float tau_g = vector_dot(&n[k], &arm[k].axis);
      arm[k].Joint_torque = -tau_g;
    }
  }
}

/**
 * @brief 角度制转弧度制函数
 */
float ator(float a) { return a * PI / 180.0f; }

/**
 * @brief 各个关节电机扭矩换算函数
 */
void Torque_Conversion(Link_StructTypeDef arm[Joint_NUM + 1]) {
  if (RoboControl_Struct.Robo_Enable) {
    /* 达妙按具体型号驱动，set_value 直接收 N·m（原来的 ÷TMAX×16384 去掉，
       符号和 Pitch 的 0.5 系数沿用旧版） */
    arm[0].Joint_torque_output = arm[0].Joint_torque;
    arm[1].Joint_torque_output = -(0.5f * arm[1].Joint_torque);
    arm[2].Joint_torque_output = -(0.5f * arm[2].Joint_torque);
    arm[3].Joint_torque_output = arm[3].Joint_torque;
    arm[4].Joint_torque_output = arm[4].Joint_torque;
    arm[5].Joint_torque_output = arm[5].Joint_torque;
    /* J7 换成 MG4010E 型号后 set_value 也收 N·m（暂按满量程 ±2.5 N·m，
       见 Motor_LINGKONG_Driver.h）。原来按 MG6010 参数换成转矩电流原始值 */
    arm[6].Joint_torque_output = arm[6].Joint_torque;
  }
}

void vector_add(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b,
                Vector_StructTypeDef *out) {
  float x = a->x + b->x;
  float y = a->y + b->y;
  float z = a->z + b->z;
  out->x = x;
  out->y = y;
  out->z = z;
}

void vector_mul(const Vector_StructTypeDef *a, float k,
                Vector_StructTypeDef *out) {
  float x = a->x * k;
  float y = a->y * k;
  float z = a->z * k;
  out->x = x;
  out->y = y;
  out->z = z;
}

float vector_dot(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b) {
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

void vector_cross(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b,
                  Vector_StructTypeDef *out) {
  float x = a->y * b->z - a->z * b->y;
  float y = a->z * b->x - a->x * b->z;
  float z = a->x * b->y - a->y * b->x;
  out->x = x;
  out->y = y;
  out->z = z;
}

void matrix_transpose(const Matrix_StructTypeDef *R,
                      Matrix_StructTypeDef *out) {
  Matrix_StructTypeDef temp;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      temp.m[i][j] = R->m[j][i];
    }
  }
  *out = temp;
}

void matrix_mul(const Matrix_StructTypeDef *A, const Matrix_StructTypeDef *B,
                Matrix_StructTypeDef *out) {
  Matrix_StructTypeDef temp;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      temp.m[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        temp.m[i][j] += A->m[i][k] * B->m[k][j];
      }
    }
  }
  *out = temp;
}

void matrix_mul_vector(const Matrix_StructTypeDef *A,
                       const Vector_StructTypeDef *B,
                       Vector_StructTypeDef *out) {
  float x = A->m[0][0] * B->x + A->m[0][1] * B->y + A->m[0][2] * B->z;
  float y = A->m[1][0] * B->x + A->m[1][1] * B->y + A->m[1][2] * B->z;
  float z = A->m[2][0] * B->x + A->m[2][1] * B->y + A->m[2][2] * B->z;
  out->x = x;
  out->y = y;
  out->z = z;
}

void Homogeneous_mul(const Homogeneous_StructTypeDef *A,
                     const Homogeneous_StructTypeDef *B,
                     Homogeneous_StructTypeDef *out) {
  Homogeneous_StructTypeDef temp;

  matrix_mul(&(A->R), &(B->R), &(temp.R));

  Vector_StructTypeDef temp_vec;
  matrix_mul_vector(&(A->R), &(B->p), &temp_vec);
  vector_add(&temp_vec, &(A->p), &(temp.p));

  *out = temp;
}

void rot_x(float theta, Matrix_StructTypeDef *out) {
  float c = cosf(theta), s = sinf(theta);
  out->m[0][0] = 1.0f;
  out->m[0][1] = 0.0f;
  out->m[0][2] = 0.0f;
  out->m[1][0] = 0.0f;
  out->m[1][1] = c;
  out->m[1][2] = -s;
  out->m[2][0] = 0.0f;
  out->m[2][1] = s;
  out->m[2][2] = c;
}

void rot_y(float theta, Matrix_StructTypeDef *out) {
  float c = cosf(theta), s = sinf(theta);
  out->m[0][0] = c;
  out->m[0][1] = 0.0f;
  out->m[0][2] = s;
  out->m[1][0] = 0.0f;
  out->m[1][1] = 1.0f;
  out->m[1][2] = 0.0f;
  out->m[2][0] = -s;
  out->m[2][1] = 0.0f;
  out->m[2][2] = c;
}

void rot_z(float theta, Matrix_StructTypeDef *out) {
  float c = cosf(theta), s = sinf(theta);
  out->m[0][0] = c;
  out->m[0][1] = -s;
  out->m[0][2] = 0.0f;
  out->m[1][0] = s;
  out->m[1][1] = c;
  out->m[1][2] = 0.0f;
  out->m[2][0] = 0.0f;
  out->m[2][1] = 0.0f;
  out->m[2][2] = 1.0f;
}

void rpy_to_rot(float roll, float pitch, float yaw, Matrix_StructTypeDef *out) {
  Matrix_StructTypeDef Rx, Ry, Rz, R_zy;
  rot_x(roll, &Rx);
  rot_y(pitch, &Ry);
  rot_z(yaw, &Rz);

  matrix_mul(&Rz, &Ry, &R_zy);
  matrix_mul(&R_zy, &Rx, out);
}

void matrix_identity(Matrix_StructTypeDef *out) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      out->m[i][j] = (i == j) ? 1.0f : 0.0f;
    }
  }
}

void vector_zero(Vector_StructTypeDef *out) {
  out->x = 0.0f;
  out->y = 0.0f;
  out->z = 0.0f;
}
