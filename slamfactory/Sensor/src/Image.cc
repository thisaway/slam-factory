#include "Image.h"

#include <cassert>

namespace sf{
    
Image::
Image(){}

Image::
Image(int r,int c,uint8_t iniValue){

    assert(r>0 and c>0);

    rows=r;
    cols=c;
    data.resize(rows,cols);

    if(iniValue==0)data=MatXui8::Zero(r,c);
    else data=MatXui8::Ones(r,c)*iniValue;
}

Image::
Image(const MatXui8& img){

    rows=img.rows();
    cols=img.cols();
    assert(rows>0 and cols>0);

    data.resize(rows,cols);
    data=img;  //Auto conversion to uint8; fail
}

Image::
Image(const Image& img){

    rows=img.rows;
    cols=img.cols;
    assert(rows>0 and cols>0);

    data.resize(rows,cols);
    data=img.data;
}

Image& Image::
operator=(const Image& img){

    rows=img.rows;
    cols=img.cols;
    assert(rows>0 and cols>0);

    data.resize(rows,cols);
    data=img.data;

    return *this;
}

Image::
~Image(){}
}