#ifndef __FT5216_H__
#define __FT5216_H__

#include "Arduino.h"
#include <Wire.h>

#define FT5216_ADDR 0x38

class FT5216 {
    public: 
        FT5216() ;

        void init() ;
        uint8_t read(uint16_t *, uint16_t *) ;

};

extern FT5216 touch;

#endif