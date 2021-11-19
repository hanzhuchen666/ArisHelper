# ARIS HELPER DOC

## 运动学和动力学简称规则
### 位置
- pp  :  3x1 点位置(position of point)  
- re  :  3x1 欧拉角(eula angle)         
- rq  :  4x1 四元数(quaternions)       
- rm  :  3x3 旋转矩阵(rotation matrix)  
- pe  :  6x1 点位置与欧拉角(position and eula angle)
- pq  :  7x1 点位置与四元数(position and quaternions)
- pm  :  4x4 位姿矩阵(pose matrix)
- ra  :  3x1 绕固定轴的旋转的指数积（rotation around axis, exponential product）
- ps  :  6x1 位移螺旋，[v;w]的速度螺旋转过theta角，也就是ps = [v;w]*theta
### 速度
- vp  :  3x1 线速度(velocity of point)
- we  :  3x1 欧拉角导数(omega in term of eula angle)
- wq  :  4x1 四元数导数(omega in term of quternions)
- wm  :  3x3 旋转矩阵导数(omega in term of rotation matrix)
- ve  :  6x1 线速度与欧拉角导数（velocity and omega in term of eula angle）
- vq  :  7x1 线速度与四元数导数(velocity and omega in term of quternions)
- vm  :  4x4 位姿矩阵导数(velocity in term of pose matrix)
- wa  :  3x1 角速度(omega)
- va  :  6x1 线速度与角速度(velocity and omega)
- vs  :  6x1 螺旋速度(velocity screw)
### 加速度
- ap  :  3x1 线加速度(acceleration of point)
- xe  :  3x1 欧拉角导导数(alpha in term of eula angle)
- xq  :  4x1 四元数导导数(alpha in term of quternions)
- xm  :  3x3 旋转矩阵导导数(alpha in term of rotation matrix)
- ae  :  6x1 线加速度与欧拉角导导数(acceleration and alpha in term of eula angle)
- aq  :  7x1 线加速度与四元数导导数(acceleration and alpha in term of quternions)
- am  :  4x4 位姿矩阵导导数(acceleration in term of pose matrix)
- xa  :  3x1 角加速度(alpha, acceleration of angle)
- aa  :  6x1 线加速度与角加速度(acceleration and alpha)
- as  :  6x1 螺旋加速度(acceleration screw)
### 动力学特征
- i3  :  3x3 惯量矩阵
- im  :  6x6 空间惯量矩阵
- iv  :  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
- cm  :  3x3 反对称矩阵

## 数学工具函数
### /src/dynamic/screw.cpp
- 位姿矩阵的逆矩阵
    - s_inv_pm(const double *pm_in, double *pm_out)
- 位姿矩阵点乘 位姿矩阵
    - s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out) 
- s_inv_pm_dot_pm(const double *inv_pm, const double *pm, double *pm_out)
- s_pm_dot_inv_pm
- 矩阵乘以向量
    - s_pm_dot_v3
- s_inv_pm_dot_v3
- 空间惯量矩阵乘螺旋加速度
- s_im_dot_as
- s_iv_dot_as
- 将向量转换为反对称矩阵
    - s_cm3(const double *a, double *cm_out)
- 三维向量叉乘 C = A x B
    - s_c3(const double *a, const double *b, double *c_out)
    - s_c3(double alpha, const double *a, const double *b, double *c_out)
- 三维向量叉乘后取反 C = - A x B
    - s_c3i(const double *a, const double *b, double *c_out)
- 向量加三维向量叉乘 C + = A x B
    - s_c3a(const double *a, const double *b, double *c_out)
    - s_c3a(double alpha, const double *a, const double *b, double *c_out) C += alpha * A x B
- 向量叉乘后变负值

### /include/dynamic/matrix.hpp
#### 行主元或列主元（可以用一维数组表示二维矩阵）
- 数字表示行主元，行元素个数
#### 功能函数
- 函数功能 （可选：固定输出大小）
    - 函数名(参数Size为先行数后列数)
    - 重载函数,一般取消Type默认为行主元
    - ...
- 矩阵打印
    - dsp(Size, Size, const T *data, Type)
    - dsp(Size, Size, const T *data)
- 矩阵输出到文件
    - dlmwrite(Size, Size, const T*data, Type, const char\*filename)
    - 
- s_sgn
- 判断是否相同
    - 数值 s_is_equal(double a, double b, double error)
    - 矩阵
        - s_is_equal(Size, )
        - 
        - 
        - 
- 创建单位阵(方阵)
    - s_eye(Size, double* A, Type) Size为行数\列数
- 绕x轴旋转矩阵 3*3
    - s_rmx(double angle, double *A, Type), angle为弧度制
- 绕y轴旋转矩阵
- 绕z轴旋转矩阵
- 求列向量模 Size*1
    - s_norm(Size, const double *x, Type)
    - s_norm(Size, const double)
- 交换两个向量的值
- 交换两个矩阵的值
- 将矩阵填充给定数值
- 将列向量取负值
    - s_iv(Size, double *x, Type)
    - s_iv(Size, double *x)
- 将列向量元素乘给定数
    - s_nv(Size, double *x, Type)
    - s_nv(Size, double *x)
- 拷贝列向量 或乘以alpha后拷贝
    - s_vc(Size, double *x, Type)
    - s_vc(Size, double *x)
    - s_vc(Size, double *x, Type)
    - s_vc(Size, double *x)
- 将列向量取负后拷贝
    - s_vi(Size, double *x, Type)
    - s_vi(Size, double *x)
-  列向量相加 或乘以alpha相加
    - s_va(Size, double *x, Type)
    - s_va(Size, double *x)
    - s_va(Size, double *x, Type)
    - s_va(Size, double *x)
-  列向量相减
    - s_vs(Size, double *x, Type)
    - s_vs(Size, double *x)
- 向量点乘 返回double值
    - s_vv(Size, double *x, Type)
    - s_vv(Size, double *x)
- 矩阵乘常量
    - s_nm(Size, double *x, Type)
    - s_nm(Size, double *x)
- 矩阵取负
    - s_im(Size, Size, double* A, Type)
    - S_im(Size, Size, double* A)
- 矩阵拷贝，或乘alpha拷贝
    - s_mc(Size, double *x, Type)
    - s_mc(Size, double *x)
    - s_mc(Size, double *x, Type)
    - s_mc(Size, double *x)
- 矩阵相加
    - s_ma(Size, double *x, Type)
    - s_ma(Size, double *x)
***
- 矩阵取负后拷贝
    - s_va(Size, double *x, Type)
    - s_va(Size, double *x)
- 矩阵相减
- 矩阵相乘后相加
- 矩阵相乘后相减
- 矩阵相乘
- 矩阵相乘后取负
- 矩阵行变换	// x_new[i] = x[p[i]]
- 矩阵行变换的逆	// x_new[p[i]] = x[i]
- 矩阵的[LLT分解][LLT]


### \src\dynamics\

## 运功学和动力学解算例程
### 抽象模型讲解
### 模型建立
### 运算


[LLT]: {https://blog.csdn.net/qq_41839222/article/details/96274251}