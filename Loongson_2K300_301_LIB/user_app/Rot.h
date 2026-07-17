#ifndef __ROT_H_
#define __Rot_H_
#include "Define.h"
#include "Image.h"

extern const float mapx[240][320];
extern const float mapy[240][320];
extern const float inv_mapx[240][320];
extern const float inv_mapy[240][320];

float Xrot_point(int x,int y);
float Yrot_point(int x,int y);
float inv_rot_x(float x,float y);
float inv_rot_y(float x,float y);
float XShowRotimg(uint8 x,uint8 y);
float YShowRotimg(uint8 x,uint8 y);
#endif /* __ROT_H_  */