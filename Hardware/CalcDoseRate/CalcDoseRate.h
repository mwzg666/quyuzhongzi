/*************************************************************


文件名:  doserate.h
作  者:  潘国义
描  述:  剂量率处理模块
修订记录:   
       2019-4-3  初始版本
       2019-4-21 代码从wifi主机移植过来，并将A,B,C值参数化
**************************************************************/


#ifndef __DOSERATE_H__
#define __DOSERATE_H__

// 每一秒被调用一次
/*
函数功能：计数平滑，并返回平滑后计数；根据平滑后计数值计算剂量率（计数为-1时，使用旧的剂量率）
注意：函数返回计数（-1为无效，请无视跳过，比如不切换量程）。
输入：（A,B,C,实时计数，旧剂量率）
输出：剂量率
*/
float SmothCpsCalcRate(float RealCPS, float OlduSvh, float *NewuSvh);
//初始化需清零，量程切换需要清零
void ClearCounter();
//初始化
void InitArr();
float CpsToUsv_h(float count);

#endif

