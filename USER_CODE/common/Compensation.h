#ifndef __Compensation__
#define __Compensation__

#include "main.h"
#include "math.h"


#define Joint_NUM 7 //关节数量
#define GRAVITY 9.80665f

/*===| 3×3矩阵结构体定义 |===*/
typedef struct {
  float m[3][3];
} Matrix_StructTypeDef;

/*===| 三维向量结构体定义 |===*/
typedef struct {
  float x;
  float y;
  float z;
} Vector_StructTypeDef;

/*===| 齐次变换矩阵结构体定义 |===*/
typedef struct {
  Matrix_StructTypeDef R;
  Vector_StructTypeDef p;
} Homogeneous_StructTypeDef;

/*===| 机械臂连杆参数结构体结构体定义 |===*/
typedef struct {
  float m;                   // 连杆质量
  Vector_StructTypeDef p_ci; // 质心在自身连杆坐标系下的位置
  Homogeneous_StructTypeDef
      T_fixed; // 关节零位时，当前连杆相对上一连杆的固定变换
  Vector_StructTypeDef axis; // 关节旋转轴
  float q_offset;            // 关节角度偏移(角度制)
  float q;                   // 关节角度(弧度制)
  float Joint_torque;        // 关节输出力矩
  float Joint_torque_output; // 关节输出力矩实际发送数据
} Link_StructTypeDef;

void link_init(Link_StructTypeDef link[Joint_NUM + 1]);

void gravity_calc(Link_StructTypeDef arm[Joint_NUM + 1]);

float ator(float a);

void vector_add(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b,
                Vector_StructTypeDef *out);

void vector_mul(const Vector_StructTypeDef *a, float k,
                Vector_StructTypeDef *out);

float vector_dot(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b);

void vector_cross(const Vector_StructTypeDef *a, const Vector_StructTypeDef *b,
                  Vector_StructTypeDef *out);

void matrix_transpose(const Matrix_StructTypeDef *R, Matrix_StructTypeDef *out);

void matrix_mul(const Matrix_StructTypeDef *A, const Matrix_StructTypeDef *B,
                Matrix_StructTypeDef *out);

void matrix_mul_vector(const Matrix_StructTypeDef *A,
                       const Vector_StructTypeDef *B,
                       Vector_StructTypeDef *out);

void Homogeneous_mul(const Homogeneous_StructTypeDef *A,
                     const Homogeneous_StructTypeDef *B,
                     Homogeneous_StructTypeDef *out);

void rot_x(float theta, Matrix_StructTypeDef *out);

void rot_y(float theta, Matrix_StructTypeDef *out);

void rot_z(float theta, Matrix_StructTypeDef *out);

void rpy_to_rot(float roll, float pitch, float yaw, Matrix_StructTypeDef *out);

void matrix_identity(Matrix_StructTypeDef *out);

void vector_zero(Vector_StructTypeDef *out);

void Torque_Conversion(Link_StructTypeDef arm[Joint_NUM + 1]);

#endif
