#include "ubxDecoder.h"

ubxDecoder::ubxDecoder(uint8_t hdr0, uint8_t hdr1,uint16_t buf_len)
{
    _hdr0=hdr0;
    _hdr1=hdr1;
    _count=0;
    _buf.resize(buf_len);
    _raw = &_buf[0];
}

int ubxDecoder::receive(uint8_t x)
{
    int ret=0;

    if( _count>=2 )
    {
        _cs[0]+=x;
        _cs[1]+=_cs[0];
    }

    _raw[_count]=x;

    switch(_count)
    {
    case	0:	if( x==_hdr0 ) {_count++;} break;
    case	1:	if( x==_hdr1 ) {_count++; _cs[0]=_cs[1]=0;} else {_count=0;ret=-1;} break;
    case	2:	//class
    case	3:	//id
    case	4:	//length L
                _count++; break;
    case	5:	//length H
                _count++;
                _length=(*(uint16_t*)&_raw[4] )+2+1+1+2; //payload + header
                if( (_length+2)>=(uint16_t)_buf.size() )//payload + header + csum
                {
                     ret=-4;
                     _count=0;	//too long frame,ignore
                }
                if(_length==6)
                {//no payload, capture checksum now
                    __cs[0]=_cs[0];
                    __cs[1]=_cs[1];
                }
                break;
    default:
        if( _count==_length-1 )
        {	//end of data frame,remain 2 bytes of checksum
            __cs[0]=_cs[0];
            __cs[1]=_cs[1];
            _count++;
        }
        else if( _count>=(_length-1+2) )
        {	//checksum received
            if( (__cs[1]==_raw[_count])&&(__cs[0]==_raw[_count-1]) )
            {// checksum ok
                _length+=2;
                ret=1;
            }
            else
            {// checksum error
                ret=-(_count-1);
            }
            _count=0;
        }
        else
        {
            if( ++_count>=(uint16_t)_buf.size() ) _count=0;	//redundant
        }
    }
    return ret;
}

const uint8_t *ubxDecoder::raw() const
{
    return _raw;
}

uint16_t ubxDecoder::length() const
{
    return _length;
}
