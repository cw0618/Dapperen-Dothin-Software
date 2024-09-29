#ifndef _ONI_TYPEHPP_H_
#define _ONI_TYPEHPP_H_

typedef uint8_t uchar;

struct Point2s{
    Point2s(){}
    Point2s(uint16_t a, uint16_t b){
      x = a;
      y = b;
    }

   Point2s& operator = (const Point2s& pt)
   {
      x = pt.x; y = pt.y;
     return *this;
   }


    uint16_t x;
    uint16_t y;
};

#endif
