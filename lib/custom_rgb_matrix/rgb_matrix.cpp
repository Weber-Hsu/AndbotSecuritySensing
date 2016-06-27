
/***************************************************************************** 
*                Copyright:  ChengDu Geeker Tech. Co., Ltd. 
* File name:      rgb_matrix.cpp 
* Description:    rgb matrix lib
* Author:       wanghui_CD
* Version:      V1.0
* Date:         2012.06.21 
* History:      none
*****************************************************************************/

extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <stdlib.h>
  #include <math.h>
  #include <string.h>
  #undef abs
}
#include "../custom_rgb_matrix/rgb_matrix.h"


/*************************************************************************                    
*  Description:
*                         rgb_matrix���캯��
* Param:   x8 -- X����8X8�����ĸ���
* Param:   y8 -- y����8X8�����ĸ���
* Param:   _data_pin  -- data ��ź�
* Param:   _clk_pin   -- clk ��ź�
* Param:   _latch_pin -- latch ��ź�
* Retval:      none 
**************************************************************************/
rgb_matrix::rgb_matrix(uint8_t x8, uint8_t y8, uint8_t _data_pin,uint8_t _clk_pin,uint8_t _latch_pin)
{
    // record pins for sw spi
    data_pin  = _data_pin;
    clk_pin   = _clk_pin;
    latch_pin = _latch_pin;
    // set ddr for sw/hw spi pins
    if((data_pin == 11) && (clk_pin == 13))
    {
        SPI.begin();
        SPI.setBitOrder(MSBFIRST);
        hw_spi = 1;
    }
    else
    {
        pinMode(clk_pin, OUTPUT);
        pinMode(data_pin, OUTPUT);
        hw_spi = 0;
    }
    pinMode(latch_pin, OUTPUT);

    // allocate screenbuffers
    single_color_buffer_size = x8 * y8 * 8;

    top_layer = (struct screens_buffer*)malloc(sizeof(struct screens_buffer));

    if((!top_layer))
    {
      return ;
    }
    top_layer->is_heap = 1;
    top_layer->buffer = (uint8_t *)malloc(single_color_buffer_size*3);
    if(!(top_layer->buffer))
    {
      free(top_layer);
      return ;
    }
    buffer = (uint8_t *)malloc(single_color_buffer_size*3);
    if(!buffer)
    {
        free(top_layer->buffer);
        free(top_layer);
        return ;
    }

    layer[0] = top_layer;
    sum_layer = 1;
    max_x = x8 * 8;
    max_y = y8 * 8;
    cursor_x = 0;
    cursor_y = max_y - 8;
    {
        int i;
        for(i=0;i<16;i++)
            param[i] = 0;
        param_num = 0;
    }
    clear();             // clear display
}

/*************************************************************************                    
*  Description:
*                         ������ͼ��(��heap�з��仺��ռ�)
* Param:   none
* Retval:  >0  ����ͼ��������
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::add_layer()
{
    struct screens_buffer* p;
    uint8_t *p_buf;
    uint16_t i,size = single_color_buffer_size*3;
    struct screens_buffer* prev;

    if(sum_layer == MAX_LAYER)
    {
        return -1;
    }
        
    p = (struct screens_buffer*)malloc(sizeof(struct screens_buffer));
    if((uint16_t)p == NULL)
    {
        return -1;
    }
    p->is_heap = 1;
    p->buffer = (uint8_t*)malloc(size);
    if((uint16_t)p->buffer == NULL)
    {
        free(p);
        return -1;
    }
    
    p->index = sum_layer;
    p->next = NULL;
    prev = layer[sum_layer - 1];
    prev->next = p;
    sum_layer ++;
    layer[sum_layer - 1] = p;
    p_buf = p->buffer;
    for(i=0; i < size; i++)
    {
        *p_buf++ = 0;
    }
    clear(sum_layer-1);
    return sum_layer-1;
}

/*************************************************************************                    
*  Description:
*                         ������ͼ��(�ɵ�����ָ������ռ�)
* Param:   p_param    ����Ļ���ռ��ָ��
* Retval:  >0  ����ͼ��������
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::add_layer(uint8_t *p_param)
{
    struct screens_buffer* p;
    uint16_t i,size = single_color_buffer_size*3;
    struct screens_buffer* prev;

    if(sum_layer == MAX_LAYER)
    {
        return -1;
    }
    
    if((uint16_t)p_param == NULL)
    {
        return -1;
    }
    
    p = (struct screens_buffer*)malloc(sizeof(struct screens_buffer));
    if((uint16_t)p == NULL)
    {
        return -1;
    }
    
    p->is_heap = 0;
    p->buffer = p_param;
    
    p->index = sum_layer;
    p->next = NULL;
    prev = layer[sum_layer - 1];
    prev->next = p;
    sum_layer ++;
    layer[sum_layer - 1] = p;

    return sum_layer - 1;
}

/*************************************************************************                    
*  Description:
*                         �Ƴ����һ��ͼ��(top���ֹ�ƶ�)
* Param:   none    
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::remove_layer(void)
{
    if(sum_layer == 1)
    {
        return -1;
    }
    else
    {
        return remove_layer(sum_layer - 1);
    }
}

/*************************************************************************                    
*  Description:
*                         �Ƴ�ָ������ŵ�һ��ͼ��(top���ֹ�ƶ�)
* Param:   index  ָ���Ƴ�ͼ��������    
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::remove_layer(uint8_t index)
{
    struct screens_buffer* p;
    struct screens_buffer* prev;
    
    uint8_t i;
    
    if(index > sum_layer -1)
    {
        return -1;
    }
    if(index == 0)
    {
        return -1;
    }
    prev = layer[index - 1];
    p    = layer[index];
    
    for(i = index; i < sum_layer; i++)
    {
        layer[i] = layer[i+1];
        layer[i]->index--;
    }
    layer[sum_layer] = 0;
    
    sum_layer--;
    prev->next = p->next;
    if(p->is_heap)
    {
        free(p->buffer);
    }
    free((uint8_t*)p);
    dirty = 1;
    return 0;
}

// sends a single byte to display buffer
/*************************************************************************                    
*  Description:
*                         �򶥲�ͼ�㻺����д��һ���ֽڵ����
* Param:   x  x����ַ    
* Param:   y  y����ַ 
* Param:   data  Ҫд������    
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_byte(uint16_t x, uint16_t y, uint8_t data, uint8_t mode, uint8_t color)
{
   return put_byte( x,  y,  data,  mode,  color, TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ�㻺����д��һ���ֽڵ����
* Param:   x  x����ַ    
* Param:   y  y����ַ 
* Param:   data  Ҫд������    
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_byte(uint16_t x, uint16_t y, uint8_t data, uint8_t mode, uint8_t color, uint8_t index)
{
    struct screens_buffer *screen;
    uint8_t *p;
    uint8_t data1,data2;
    uint16_t base;
    uint8_t cross_tail = 0;
    uint8_t y8 = max_y/8;
    
    if(index > sum_layer - 1)
    {
        return -1;
    }
    if((x >= max_x) || (y >= max_y))
    {
        return -1;
    }
    if((y + 8 < max_y) && (y%8 != 0))
    {
        cross_tail = 1;
    }
    
    screen = layer[index];
    p = screen->buffer;
    data1 = data << (y%8);
    data2 = data >>(8- y%8);
    if(color & BLUE)
    {
        if (mode == MULTIPLY)
        {
            p[y/8 + x*y8] = p[y/8 + x*y8] | data1;
            if(cross_tail)
            {
                p[y/8 + 1 + x*y8] = p[y/8 + 1 + x*y8] | data2;
            }
        }
        else
        {
            p[y/8 + x*y8] = data1;
            if(cross_tail)
            {   
                p[y/8 + 1 + x*y8] = data2;
            }
        }
    }
    if(color & GREEN)
    {
        base = single_color_buffer_size;
        if (mode == MULTIPLY)
        {
            p[base + y/8 + x*y8] = p[base + y/8 + x*y8] | data1;
            if(cross_tail)
            {
                p[base + y/8 + 1 + x*y8] = p[base + y/8 + 1 + x*y8] | data2;
            }
        }
        else
        {
            p[base + y/8 + x*y8] = data1;
            if(cross_tail)
            {
                p[base + y/8 + 1 + x*y8] = data2;
            }
        }        
    }
    if(color & RED)
    {
        base = single_color_buffer_size * 2;
        if (mode == MULTIPLY)
        {
            p[base + y/8 + x*y8] = p[base + y/8 + x*y8] | data1;
            if(cross_tail)
            {
                p[base + y/8 + 1 + x*y8] = p[base + y/8 + 1 + x*y8] | data2;
            }
        }
        else
        {
            p[base + y/8 + x*y8] = data1;
            if(cross_tail)
            {
                p[base + y/8 + 1 + x*y8] = data2;
            }
        }
    }
    int i;
    
    dirty = 1;
    return 0;
}

/*************************************************************************                    
*  Description:
*                         �򶥲�ͼ�㻺����д��һ��ASSIC�ַ�
* Param:   x  x����ַ    
* Param:   y  y����ַ 
* Param:   asc  Ҫд���ַ��ASCII��
* Param:   rotation  0 ����ת 1,��ʱ��90�ȷ�ת
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_char(uint16_t x,uint16_t y, uint8_t asc,uint8_t rotation, uint8_t mode, uint8_t color)
{
    put_char(x, y, asc, rotation, mode, color, TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ�㻺����д��һ��ASSIC�ַ�
* Param:   x  x����ַ    
* Param:   y  y����ַ 
* Param:   asc  Ҫд���ַ��ASCII��
* Param:   rotation  0 ����ת 1,��ʱ��90�ȷ�ת
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_char(uint16_t x,uint16_t y, uint8_t asc, uint8_t rotation, uint8_t mode, uint8_t color, uint8_t index)
{
    //uint8_t* p;
    uint8_t i,base;
    if((x >= max_x) || (y >= max_y))
    {
        return -1;
    }
    if((asc < ' ') || (asc > '~'))
    {
        return -1;
    }
    base = asc - ' ';

    if(rotation == NO_ROT)
    {
        if(mode == COVER)
        {
            clear(x,y,x+4,y+7,index);
        }
        for(i = 0; i < 5; i++)
        {
            if(x+i > max_x-1)
                break;
            put_byte(x+i,y,ASCII[base][i],mode,color,index);
        
        }
    }
    else if(rotation == ANTI_90)
    {
        uint8_t tmp,j,Y;
        if(mode == COVER)
        {
            clear(x,y,x+7,y+4,index);
        }
        for(i = 0; i < 8; i++)
        {
            tmp = 0;
            for(j=0;j<5;j++)
            {
                Y = !(!(ASCII[base][j] & (1<<(7-i))));
                tmp |= Y<<j;
            }
            if(x+i > max_x-1)
                break;
            put_byte(x+i,y,tmp,MULTIPLY,color,index);
        
        }        
    }
    dirty = 1;
    return 0;
}

// syncs screen of display with buffer
/*************************************************************************                    
*  Description:
*                         �Ѹ�ͼ�㻺���е����д����ͼ��
* Param:   none
* Retval:  nove
**************************************************************************/
void rgb_matrix::sync()
{
    uint8_t j;
    uint8_t* p_buf;
    uint16_t i,size = single_color_buffer_size * 3;
    
    p_buf = layer[0]->buffer;

    for(i = 0; i < size; i++)
    {
        buffer[i] = p_buf[i];
		for(j = 1;j < sum_layer; j++)
        {
            
            buffer[i] |= layer[j]->buffer[i];
        }
    }
}

// clears screens and buffers
/*************************************************************************                    
*  Description:
*                         ���ָ��ͼ�㻺����������
* Param:   index  ͼ�������    
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::clear(uint8_t index)
{
    uint8_t *p, i;
    uint16_t size = single_color_buffer_size * 3;
    struct screens_buffer *screen;
    if(index > sum_layer - 1)
    {
        return -1;
    }

    screen = layer[index];
    p = screen->buffer;
    
    for(i=0;i<size;i++)
    {
        *p++ = 0;
    }
    dirty = 1;
    return 0;
}

/*************************************************************************                    
*  Description:
*                         ���ָ��ͼ�㻺����ָ������λ�ô��������
* Param:   x0 y0 ָ�����ε�һ������    
* Param:   x1 y1 ָ������(x0,y0)�ĶԽǶ���
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::clear(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint8_t index)
{
    uint8_t *p, i, j, y8=max_y/8,cross_tail,tmp;
    uint16_t size = single_color_buffer_size * 3;
    int16_t tmp_x,tmp_y, base;
    struct screens_buffer *screen;
    int start_x,start_y,end_x,end_y;

    if((x0 >= max_x) || (x1 >= max_x) || (y0 >= max_y) || (y1 >= max_y))
    {
        return -1;
    }
    if(index > sum_layer - 1)
    {
        return -1;
    }

    start_x = (x0>x1)?x1:x0;
    start_y = (y0>y1)?y1:y0;
    end_x = x0 + x1 - start_x;
    end_y = y0 + y1 - start_y;

    screen = layer[index];
    p = screen->buffer;

    if((start_y/8) == (end_y/8))
    {
        cross_tail = 0;
    }
    else
    {
        cross_tail = 1;
    }
    
    for(i=0; i< end_x-start_x+1; i++)
    {
        tmp_x = start_x+i;
        tmp_y = start_y;
        if(!cross_tail)
        {
            tmp = 0xff;
            for(j=0; j< end_y-start_y+1; j++)
            {
                tmp &= ~(1<<(tmp_y%8+j));
            }
            base = 0;

            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size * 2;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
        }
        else
        {
            tmp = 0xff;
            for(j=0; j < 8-start_y%8; j++)
            {
                tmp &= ~(1<<(start_y%8+j));
            }
            base = 0;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size * 2;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
        }
        tmp_y += 8 - tmp_y%8;
        
        while(tmp_y + 7 <= end_y)
        {
            base = 0;
            p[base + tmp_x*y8 + tmp_y/8] = 0;
            base = single_color_buffer_size;
            p[base + tmp_x*y8 + tmp_y/8] = 0;
            base = single_color_buffer_size * 2;
            p[base + tmp_x*y8 + tmp_y/8] = 0;
            tmp_y+=8;
        }
        if(tmp_y <= end_y)
        {
            tmp=0xff;
            for(j = 0; j < end_y-tmp_y+1;j++)
            {
                tmp &= (~(1<<j));
            }
            base = 0;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
            base = single_color_buffer_size * 2;
            p[base + tmp_x*y8 + tmp_y/8] = p[base + tmp_x*y8 + tmp_y/8] & tmp;
        }
    }
    dirty = 1;
    return 0;
}

/*************************************************************************                    
*  Description:
*                         �������ͼ�㻺����������
* Param:   none
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::clear(void)
{
    uint8_t i;
    for(i = 0; i< sum_layer; i++)
    {
        clear(i);
    }
}

/******************************************************************************                    
*  Description:
*                         ��ʾ����(���ú󽫽���˺������ѭ��)
* Param:   display_hook  ��ʾ����Ļص�����ָ�룬��ˢ�µļ�϶������ ����0������
* Retval:  none
*******************************************************************************/
void rgb_matrix::display(void (*display_hook)())
{
    uint16_t j;
    int i;
    uint8_t *p,tmp;
    uint8_t y8 = max_y/8;
    uint16_t base_b,base_g,base_r;
    base_b = (uint16_t)&buffer[0];
    base_g = (uint16_t)&buffer[single_color_buffer_size];
    base_r = (uint16_t)&buffer[single_color_buffer_size*2];
    
    while(1)
    {
        if(dirty)
        {
            sync();
            dirty = 0;
        }
        p = buffer;
        
        for(j=0; j < max_x; j++)
        {
            digitalWrite(latch_pin,LOW);
            //blue
            for(i = y8 - 1; i >= 0; i--)
            {
                p = (uint8_t*)(base_b+j*y8);
                if(hw_spi)
                {
                    SPI.transfer(p[i]);
                }
                else
                {
                    shiftOut(data_pin,clk_pin,MSBFIRST,p[i]);
                }

                p = (uint8_t*)(base_g+j*y8);
                if(hw_spi)
                {
                    SPI.transfer(p[i]);
                }
                else
                {
                    shiftOut(data_pin,clk_pin,MSBFIRST,p[i]);                
                }
                
                p = (uint8_t*)(base_r+j*y8);
                if(hw_spi)
                {
                    SPI.transfer(p[i]);
                }
                else
                {
                    shiftOut(data_pin,clk_pin,MSBFIRST,p[i]);
                }
            }

            for(i = 0;i < (max_x-1-j)/8; i++)
            {
                if(hw_spi)
                {
                    SPI.transfer(0);
                }
                else
                {
                    shiftOut(data_pin,clk_pin,MSBFIRST,0);
                }
            }
            
            tmp = j % 8;
            if(hw_spi)
            {
                SPI.transfer(1<<tmp);
            }
            else
            {
                shiftOut(data_pin,clk_pin,MSBFIRST,1<<tmp);
            }
            //com
            for(i = 0;i < j/8; i++)
            {
                if(hw_spi)
                {
                    SPI.transfer(0);
                }
                else
                {
                    shiftOut(data_pin,clk_pin,MSBFIRST,0);
                }
            }
            digitalWrite(latch_pin,HIGH);
            
            if(((uint16_t)display_hook) != NULL)
            {
                (*display_hook)();
            }
        }
	break; // added by WeberHsu 20160602
    }
}

/*************************************************************************                    
*  Description:
*                         ������Ա����ʹ�ú���
* Param:   none
* Retval:  none
**************************************************************************/
void rgb_matrix::debug()
{

}

/*************************************************************************                    
*  Description:
*                         ���ù��λ��
* Param:   x x���    
* Param:   y y���
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::set_cur(uint16_t x,uint16_t y)
{
    if((x >= max_x) || (y >= max_y))
    {
        return -1;
    }
    cursor_x = x;
    cursor_y = y;
}

/*************************************************************************                    
*  Description:
*                         ��������õ���ʼλ��
* Param:   rotation =0 ���õ�����ת��ʾʱ�Ĺ���ʼλ��
                    =1 ���õ���ʱ�뷭ת90����ʾ�ǵĹ���ʼλ�� 
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::set_cur(uint8_t rotation)
{
    if(rotation == NO_ROT)
    {
        cursor_x = 0;
        cursor_y = max_y - 8;
    }
    else if(rotation == ANTI_90)
    {
        cursor_x = 0;
        cursor_y = 0;
    }
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��ӡһ��16bit���(10������ʽ)
* Param:   data  ��Ҫ��ӡ�����
* Param:   rotation 0 �ַ���ת��1�ַ���ʱ����ת90��
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::print(uint16_t data,uint8_t rotation)
{
    return print(data,rotation,COVER,RED,TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��ӡһ���ַ�
* Param:   �ַ���׵�ַ 
* Param:   rotation 0 �ַ���ת���ַ���ʱ����ת90��
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::print(const char p[],uint8_t rotation)        //string
{
    return print(p,rotation,COVER,RED,TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ�㻺���д�ӡ16bit���(10����)
* Param:   data  ��Ҫ��ӡ�����
* Param:   rotation 0 �ַ���ת���ַ���ʱ����ת90��
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::print(uint16_t data, uint8_t rotation,/* uint8_t dir,*/ uint8_t mode,uint8_t color,uint8_t index)
{
    uint16_t i,len = 1,div=10000;
    uint16_t tmp = data;

    if((tmp/10000) != 0)
    {
        len = 5; 
        div =10000;
    }
    else if((tmp/1000) != 0)
    {
        len = 4; 
        div =1000;
    }
    else if((tmp/100) != 0)
    {
        len = 3; 
        div =100;
    }
    else if((tmp/10) != 0)
    {
        len = 2; 
        div = 10;
    }
    else
    {
        len = 1; 
        div = 1;
    }

    if(rotation == NO_ROT)
    {
        for(i=0;i<len;i++)
        {
            if(cursor_x + 4 > max_x - 1)
            {
                cursor_x = 0;
                if(cursor_y >= 8)
                {
                    cursor_y -= 8;
                }
                else
                {
                    cursor_y = max_y - 8;
                }
            }
            put_char(cursor_x,cursor_y,((tmp/div) % 10)+ '0',rotation,mode,color,index);
            div/=10;
            if(cursor_x > max_x - 1)
            {
                cursor_x = 0;
                if(cursor_y >= 8)
                {
                    cursor_y -= 8;
                }
                else
                {
                    cursor_y = max_y - 8;
                }
            }
            else
            {
                cursor_x += 5;
            }
        }
    }
    else if(rotation == ANTI_90)
    {
        for(i=0;i<len;i++)
        {
            if(cursor_y + 4 > max_y - 1)
            {
                cursor_y = 0;
                if(cursor_x < max_x-8)
                {
                    cursor_x += 8;
                }
                else
                {
                    set_cur(rotation);
                }
            }
            put_char(cursor_x,cursor_y,((tmp/div) % 10)+ '0',rotation,mode,color,index);
            div/=10;
            if(cursor_y > max_y - 1)
            {
                cursor_y = 0;
                if(cursor_x < max_x - 8)
                {
                    cursor_x += 8;
                }
                else
                {
                    set_cur(rotation);
                }
            }
            else
            {
                cursor_y += 5;
            }
        }
    }
    return 0;
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ�㻺���д�ӡһ���ַ�
* Param:   �ַ���׵�ַ 
* Param:   rotation 0 �ַ���ת���ַ���ʱ����ת90��
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::print(const char p[],uint8_t rotation,/* uint8_t dir, */uint8_t mode,uint8_t color,uint8_t index)        //string
{
    uint16_t len = 0,i;
    const char *ps;
    ps = p;
    while(*ps++ != 0)
    {
        len++;
    }
    if(len > 255)
    {
        return -1;
    }

    if(rotation == NO_ROT)
    {
        for(i=0;i<len;i++)
        {
            if(cursor_x + 4 > max_x - 1)
            {
                cursor_x = 0;
                if(cursor_y >= 8)
                {
                    cursor_y -= 8;
                }
                else
                {
                    set_cur(rotation);
                }
            }
            put_char(cursor_x,cursor_y,p[i],rotation,mode,color,index);
            if(cursor_x > max_x - 1)
            {
                cursor_x = 0;
                if(cursor_y >= 8)
                {
                    cursor_y -= 8;
                }
                else
                {
                    set_cur(rotation);
                }
            }
            else
            {
                cursor_x += 5;
            }
        }
    }
    else if(rotation == ANTI_90)
    {
        for(i=0;i<len;i++)
        {
            if(cursor_y + 4 > max_y - 1)
            {
                cursor_y = 0;
                if(cursor_x+8 < max_x)
                {
                    cursor_x += 8;
                }
                else
                {
                    set_cur(rotation);
                }
            }
            put_char(cursor_x,cursor_y,p[i],rotation,mode,color,index);
            cursor_y += 5;
        }
    }
    return 0;
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��һ����
* Param:   x x���
* Param:   y y���
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x,int16_t y)  //point
{
    return plot(x,y,(uint8_t)RED,(uint8_t)TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��һ����
* Param:   x0 y0 �ߵ���ʼ���
* Param:   x1 y1 �ߵ���ֹ���
* Param:   add_dot 1 �����ᴦ�õ㲹��  0 �����ᴦ���õ㲹��
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x0,int16_t y0,int16_t x1,int16_t y1,uint8_t add_dot)  //line
{
    return plot(x0,y0,x1,y1,add_dot,RED,TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��һ����������
* Param:   x0 �������ʼx���
* Param:   x1 �������ֹx���
* Param:   y[] ����[x0,x1]��Χ�ڶ�Ӧ��y�������
* Param:   add_dot 1 �����ᴦ�õ㲹��  0 �����ᴦ���õ㲹��
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x0,int16_t x1,int16_t y[],uint8_t add_dot) //function
{
    return plot(x0,x1,y,add_dot,(uint8_t)RED,(uint8_t)TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��һ����Բ
* Param:   x y Բ�����
* Param:   a  x����İ��᳤
* Param:   b  y����İ��ܳ�
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::ellipse(int16_t x, int16_t y, int16_t a, int16_t b)
{
    return ellipse(x,y,a,b,RED,TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         �ڶ����ú�ɫ��һ������
* Param:   x0 y0 ���ε�һ���������
* Param:   x1 y1 ���ε�(x0,y0)�ĶԽǶ������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1) //rectangle
{
    rectangle(x0,y0,x1,y1,(uint8_t)RED,(uint8_t)TOP_LAYER);
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ���ϻ�һ����
* Param:   x x���
* Param:   y y���
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x, int16_t y, uint8_t color, uint8_t index)  //point
{
    uint8_t *p,data;
    uint8_t y8 = max_y/8;
    uint16_t base;
    struct screens_buffer* screen;
    
    if(index > sum_layer - 1)
    {
        return -1;
    }
    if((x >= max_x) || (y >= max_y))
    {
        return -1;
    }
    
    data = 1 << (y%8);
    screen = layer[index];
    p = screen->buffer;

    if(color & BLUE)
    {
        p[y/8 + x*y8] = p[y/8 + x*y8] | data;
    }
    if(color & GREEN)
    {
        base = single_color_buffer_size;
        p[base + y/8 + x*y8] = p[base + y/8 + x*y8] | data;
    }
    if(color & RED)
    {
        base = single_color_buffer_size * 2;
        p[base + y/8 + x*y8] = p[base + y/8 + x*y8] | data;
    }
    dirty = 1;
    return 0;
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ���ϻ�һ����
* Param:   x0 y0 �ߵ���ʼ���
* Param:   x1 y1 �ߵ���ֹ���
* Param:   add_dot 1 �����ᴦ�õ㲹��  0 �����ᴦ���õ㲹��
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x0,int16_t y0,int16_t x1,int16_t y1, uint8_t add_dot, uint8_t color, uint8_t index)  //line
{
    int16_t i;
    int16_t n,start_x,start_y,end_x,end_y;
    float k;
    if((x0 >= max_x) || (x1 >= max_x) || (y0 >= max_y) || (y1 >= max_y))
    {
        return -1;
    }
    start_x = x0>x1?x1:x0;
    end_x = x0 + x1 - start_x;

    if(x0 == x1)
    {
        start_y = y0>y1?y1:y0;
        end_y = y0 + y1 - start_y;
        if(y0 >= y1)
        {
            n = y0 - y1 + 1;
        }
        else
        {
            n = y1 - y0 + 1;
        }
    }
    else
    {
        if(start_x == x0)
        {
            start_y = y0;
        }
        else
        {
            start_y = y1;
        }
        
    }
    
    if(x0 == x1)
    {
        for(i=0;i<n;i++)
        {
            plot(start_x,start_y+i,color,index);
        }
    }
    else
    {
        int y_t,y_n,tmp_y,tmp_x;
        k = (y1-y0)/1.0/(x1-x0);
        y_t = y_n = start_y;
        for(tmp_x=start_x;tmp_x <= end_x; tmp_x++)
        {
            
            y_n = k*(tmp_x-start_x) + start_y+0.5;
            plot(tmp_x, y_n, color, index);
            if(add_dot)
            {
                if(k > 0)
                {
                    if(y_n > y_t + 1) //inc
                    {
                        //plot(tmp_x-1, y_t, tmp_x-1, y_t + (y_n-y_t)/2.0-0.5,1,color,index);
                        //plot(tmp_x,   y_n, tmp_x, y_n-(y_n-y_t)/2.0 + 0.5,1,color,index);
                        
                        tmp_y = y_n-1;
                        do
                        {
                            plot(tmp_x,tmp_y,color,index);
                            tmp_y--;
                        }while(tmp_y == y_t+1);
                        
                    }
                }
                else
                {
                    if(y_n < y_t - 1)  //dec
                    {
                        
                        tmp_y = y_n+1;
                        do
                        {
                            plot(tmp_x,tmp_y,color,index);
                            tmp_y++;
                        }while(tmp_y == y_t-1);
                        
                        //plot(tmp_x-1, y_t, tmp_x-1, y_t - (y_t-y_n)/2.0 + 0.5,1,color,index);
                        //plot(tmp_x,   y_n, tmp_x,   y_n + (y_t-y_n)/2.0 - 0.5,1,color,index);
                    }
                }
            }
            y_t = y_n;
        }
    }
    return 0;
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ���ϻ�һ����������
* Param:   x0 �������ʼx���
* Param:   x1 �������ֹx���
* Param:   y[] ����[x0,x1]��Χ�ڶ�Ӧ��y�������
* Param:   add_dot 1 �����ᴦ�õ㲹��  0 �����ᴦ���õ㲹��
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::plot(int16_t x0, int16_t x1, int16_t y[], uint8_t add_dot, uint8_t color, uint8_t index) //function
{
    int16_t n, i;
    int16_t len = 0;
    uint8_t x_is_inc,y_is_inc;
    
    if((x0 >= max_x) || (x1 >= max_x))
    {
        return -1;
    }
    
    n = (x0 > x1)?(x0 - x1 + 1):(x1 - x0 + 1);
    
    x_is_inc = (x0>x1)?0:1;
    for(i = 0; i< n; i++)
    {
        int y_t,y_n,tmp_y,tmp_x;
        y_t = y_n = y[0];
        if(x_is_inc)
        {
            for(tmp_x=x0;tmp_x <= x1; tmp_x++)
            {
                tmp_y = y[tmp_x-x0];
                plot(tmp_x,tmp_y,color,index);
                if(add_dot)
                {
                    y_n = tmp_y;
                    y_is_inc = (y_n > y_t)?1:0;
                    if(y_is_inc)
                    {
                        if(y_n-y_t >= 1)
                        {
                            plot(tmp_x-1, y_t, tmp_x-1, y_t + (y_n-y_t)/2.0-0.5,1);
                            plot(tmp_x,   y_n, tmp_x, y_n-(y_n-y_t)/2.0 + 0.5,1);
                        }
                    }
                    else
                    {
                        if(y_t-y_n >= 1)
                        {
                            plot(tmp_x-1, y_t, tmp_x-1, y_t - (y_t-y_n)/2.0 + 0.5,1);
                            plot(tmp_x,   y_n, tmp_x,   y_n+(y_t-y_n)/2.0 - 0.5,1);
                        }
                    }
                    y_t = y_n;
                }
            }
        }
        else
        {
            for(tmp_x=x0; tmp_x >= x1; tmp_x--)
            {
                tmp_y = y[x0-tmp_x];
                plot(tmp_x,tmp_y,color,index);
                
                if(add_dot)
                {
                    y_n = tmp_y;
                    y_is_inc = (y_n > y_t)?1:0;
                    if(y_is_inc)
                    {
                        if(y_n-y_t >= 1)
                        {
                            plot(tmp_x+1, y_t, tmp_x+1, y_t + (y_n-y_t)/2.0-0.5,1);
                            plot(tmp_x,   y_n, tmp_x, y_n-(y_n-y_t)/2.0 + 0.5,1);
                        }
                    }
                    else
                    {
                        if(y_t-y_n >= 1)
                        {
                            plot(tmp_x+1, y_t, tmp_x+1, y_t - (y_t-y_n)/2.0 + 0.5,1);
                            plot(tmp_x,   y_n, tmp_x,   y_n+(y_t-y_n)/2.0 - 0.5,1);
                        }
                    }
                    y_t = y_n;
                }
            }
        }
    }
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ���ϻ�һ����Բ
* Param:   x y Բ�����
* Param:   a  x����İ��᳤
* Param:   b  y����İ��ܳ�
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::ellipse(int16_t x, int16_t y, int16_t a, int16_t b, uint8_t color, uint8_t index)  //ellipse
{
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ���ϻ�һ������
* Param:   x0 y0 ���ε�һ���������
* Param:   x1 y1 ���ε�(x0,y0)�ĶԽǶ������
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int  rgb_matrix::rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color, uint8_t index) //rectangle
{
}

/*************************************************************************                    
*  Description:
*                         ͼ��ƽ�ƺ���
* Param:   dir   ƽ�Ʒ��� UP DOWN LEFT RIGHT
* Param:   step  ƽ�ƵĲ���
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::move(uint8_t dir, uint16_t step, uint8_t index)
{
    uint8_t *p, *pt=NULL, tmp[3],ret;
    int16_t i,j,k;
    uint8_t x8 = max_x/8,y8 = max_y/8;
    
    struct screens_buffer* screen;
    
    if(index > sum_layer - 1)
    {
        return -1;
    }
    if((dir != LEFT) && (dir != RIGHT)&& (dir != UP) &&(dir != DOWN))
    {
        return -1;
    }

    if(dir == LEFT)
    {
        step = step % max_x;
        if(step == 0)
        {
            return 0;
        }
        if(step != 1)
        {
            pt = (uint8_t *)malloc(step*y8*3);
        }
    }
    else  if(dir == RIGHT)
    {
        step = step % max_x;
        if(step == 0)
        {
            return 0;
        }
        if(step != 1)
        {
            pt = (uint8_t *)malloc(step*y8*3);
        }
    }
    else if (dir == UP)
    {
        step = step % max_y;
        if(step == 0)
        {
            return 0;
        }
    }
    else if(dir == DOWN)
    {
        step = step % max_y;
        if(step == 0)
        {
            return 0;
        }
    }
    
    screen = layer[index];
    p = screen->buffer;
    
    if(dir == LEFT)
    {
        if(step == 1)
        {
            for(i = 0; i < y8; i++)
            {
                tmp[0] = p[i];
                tmp[1] = p[single_color_buffer_size + i];
                tmp[2] = p[single_color_buffer_size*2 + i];
                for(j = 0; j < x8*8 - 1; j++)
                {
                    p[j*y8 + i] = p[(j+1)*y8 + i];
                    p[single_color_buffer_size + y8*j + i] = p[single_color_buffer_size + (j+1)*y8 + i];
                    p[single_color_buffer_size*2 + y8*j + i] = p[single_color_buffer_size*2 + (j+1)*y8 + i];
                }
                p[(max_x-1)*y8 + i] = tmp[0];
                p[single_color_buffer_size + (max_x-1)*y8 + i] = tmp[1];
                p[single_color_buffer_size*2 + (max_x-1)*y8 + i] = tmp[2];
            }
            dirty = 1;
            return 0;
        }
        else
        {
            if((uint16_t)pt == NULL)
            {
                
                for(i = 0; i < step; i++)
                {
                    move(dir, 1, index);
                }
                return 0;
            }
            
            for(i = 0; i< y8; i++)
            {
                for(k=0; k<step; k++)
                {
                    pt[k*y8 + i] = p[k*y8+i];
                    pt[step*y8 + k*y8 + i] = p[single_color_buffer_size  + k*y8 +i];
                    pt[step*y8*2 + k*y8 + i] = p[single_color_buffer_size*2 + k*y8 +i];
                }
                for(j = 0; j < x8*8 - 1; j++)
                {
                    p[y8*j + i] = p[y8*j + i + step*y8];
                    p[single_color_buffer_size + y8*j + i] = p[single_color_buffer_size + y8*j + i  + step*y8];
                    p[single_color_buffer_size*2 + y8*j + i] = p[single_color_buffer_size*2 + y8*j + i + step*y8];
                }
                for(k=0;k<step;k++)
                {
                    p[(x8*8-step)*y8 + i + k*y8] = pt[k*y8 + i];
                    p[single_color_buffer_size + (x8*8-step)*y8 + i + k*y8] = pt[step*y8 + k*y8 + i];
                    p[single_color_buffer_size*2 + (x8*8-step)*y8 + i + k*y8] = pt[step*y8*2 + k*y8 + i];
                    //hook(pt[k*y8+i]);hook(pt[step*y8 +k*y8+ i]);hook(pt[step*y8*2+k*y8 + i]);
                }
            }
        }
    }
    else if(dir == RIGHT)
    {
        if(step == 1)
        {
            for(i = 0; i< y8; i++)
            {
                
                tmp[0] = p[(max_x-1)*y8 + i];
                tmp[1] = p[single_color_buffer_size + (max_x-1)*y8 + i];
                tmp[2] = p[single_color_buffer_size*2 + (max_x-1)*y8 + i];
                
                for(j = x8*8 - 1; j > 0; j--)
                {
                    p[y8*j + i] = p[y8*j + i - step*y8];
                    p[single_color_buffer_size + y8*j + i] = p[single_color_buffer_size + y8*j + i - step*y8];
                    p[single_color_buffer_size*2 + y8*j + i] = p[single_color_buffer_size*2 + y8*j + i - step*y8];
                }
                p[i] = tmp[0];
                p[single_color_buffer_size + i] = tmp[1];
                p[single_color_buffer_size*2 + i] = tmp[2];
            }
            dirty = 1;
            return 0;
        }
        else
        {
            if((uint16_t)pt == NULL)
            {
                for(i = 0; i< step; i++)
                {
                    move(dir, 1, index);
                }
                return 0;
            }
            
            for(i = 0; i< y8; i++)
            {
                for(k=0;k<step;k++)
                {
                    pt[k*y8 + i] = p[(x8*8-step)*y8 + i + k*y8];
                    pt[step*y8 + k*y8 + i] = p[single_color_buffer_size + (x8*8-step)*y8 + i + k*y8];
                    pt[step*y8*2 + k*y8 + i] = p[single_color_buffer_size*2 + (x8*8-step)*y8 + i + k*y8];
                    //hook(pt[k*y8+i]);hook(pt[step*y8 +k*y8+ i]);hook(pt[step*y8*2+k*y8 + i]);
                }
                for(j = max_x - 1; j >= step; j--)
                {
                    p[y8*j + i] = p[y8*(j-step) + i];
                    p[single_color_buffer_size + y8*j + i] = p[single_color_buffer_size + y8*(j-step) + i];
                    p[single_color_buffer_size*2 + y8*j + i] = p[single_color_buffer_size*2 + y8*(j-step) + i];
                }
                
                for(k=0; k<step; k++)
                {
                    p[k*y8+i] = pt[k*y8 + i];
                    p[single_color_buffer_size  + k*y8 +i] =  pt[step*y8 + k*y8 + i];
                    p[single_color_buffer_size*2 + k*y8 +i] = pt[step*y8*2 + k*y8 + i];
                }
            }
            
        }
    }
    else if(dir == UP)
    {
        
        if(step == 1)
        {
            for(i = 0; i< max_x; i++)
            {
                uint8_t Y,Yt[3];
                Yt[0] = !(!(p[(i+1)*y8 - 1]&0x80));
                Yt[1] = !(!(p[single_color_buffer_size + (i+1)*y8 - 1]&0x80));
                Yt[2] = !(!(p[single_color_buffer_size*2 + (i+1)*y8 - 1]&0x80));
                for(j=y8-1;j > 0;j--)
                {
                    Y = !(!(p[i*y8 + j-1]&0x80));
                    p[i*y8 + j] <<= 1 ;
                    p[i*y8 + j] |= Y;
                    
                    Y = !(!(p[single_color_buffer_size + i*y8 + j-1]&0x80));
                    p[single_color_buffer_size + i*y8 + j] <<= 1 ;
                    p[single_color_buffer_size + i*y8 + j] |= Y;
                    
                    Y = !(!(p[single_color_buffer_size*2 + i*y8 + j-1]&0x80));
                    p[single_color_buffer_size*2 + i*y8 + j] <<= 1 ;
                    p[single_color_buffer_size*2 + i*y8 + j] |= Y;
                }
                p[i*y8] <<= 1;
                p[i*y8] |= Yt[0];

                p[single_color_buffer_size + i*y8] <<= 1;
                p[single_color_buffer_size + i*y8] |= Yt[1];
                
                p[single_color_buffer_size*2 + i*y8] <<= 1;
                p[single_color_buffer_size*2 + i*y8] |= Yt[2];
                
            }
            dirty = 1;
            return 0;
        }
        else
        {
            for(i = 0; i < step; i++)
            {
                move(dir,1,index);
            }
        }
        
    }
    else if(dir == DOWN)
    {
        if(step == 1)
        {
            for(i = 0; i< max_x; i++)
            {
                uint8_t Y,Yt[3];
                
                Yt[0] = p[i*y8]&0x01;
                Yt[1] = p[single_color_buffer_size + i*y8] & 0x01;
                Yt[2] = p[single_color_buffer_size*2 + i*y8] & 0x01;
                
                for(j=0; j < y8-1;j++)
                {
                    Y = p[i*y8 + j+1]&0x01;
                    p[i*y8 + j] >>= 1 ;
                    p[i*y8 + j] |= Y*0x80;
                    
                    Y = p[single_color_buffer_size + i*y8 + j+1]&0x01;
                    p[single_color_buffer_size + i*y8 + j] >>= 1 ;
                    p[single_color_buffer_size + i*y8 + j] |= Y*0x80;
                    
                    Y = p[single_color_buffer_size*2 + i*y8 + j+1]&0x01;
                    p[single_color_buffer_size*2 + i*y8 + j] >>= 1 ;
                    p[single_color_buffer_size*2 + i*y8 + j] |= Y*0x80;
                }
                p[(i+1)*y8 - 1] >>= 1;
                p[(i+1)*y8 - 1] |= Yt[0]*0x80;

                p[single_color_buffer_size + (i+1)*y8 - 1] >>= 1;
                p[single_color_buffer_size + (i+1)*y8 - 1] |= Yt[1]*0x80;
                
                p[single_color_buffer_size*2 + (i+1)*y8 - 1] >>= 1;
                p[single_color_buffer_size*2 + (i+1)*y8 - 1] |= Yt[2]*0x80;
            }
            dirty = 1;
            return 0;
        }
        else
        {
            for(i = 0; i < step; i++)
            {
                move(dir,1,index);
            }
        }
    }
    free(pt);
    dirty = 1;
    return 0;
}

/*************************************************************************                    
*  Description:
*                         ͼ����һ��ֱ��ƽ��
* Param:   x0 y0 �ߵ���ʼ���
* Param:   x1 y1 �ߵ���ֹ���
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::move(int16_t x0,int16_t y0,int16_t x1,int16_t y1, uint8_t index)
{
    uint8_t *p,data;
    uint8_t y8 = max_y/8;
    uint16_t base;
    struct screens_buffer* screen;
    
    if((x0 >= max_x) || (x1 >= max_x) || (y0 >= max_y) || (y1 >= max_y))
    {
        return -1;
    }
}

/*************************************************************************                    
*  Description:
*                         ͼ����һ����������ƽ��
* Param:   x0 �������ʼx���
* Param:   x1 �������ֹx���
* Param:   y[] ����[x0,x1]��Χ�ڶ�Ӧ��y�������
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::move(int16_t x0,int16_t x1,int16_t y[],uint8_t index)
{
}
/*************************************************************************                    
*  Description:
*                         �ض�ͼ����ض���������ƽ�ƺ���
* Param:   x0 y0 ���������һ����ʼ���
* Param:   x1 y1 �����������һ���Խ����
* Param:   dir   ƽ�Ʒ��� UP DOWN LEFT RIGHT
* Param:   step  ƽ�ƵĲ���
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::move(uint16_t x0 ,uint16_t y0, uint16_t x1, uint16_t y1, uint8_t dir, uint16_t step, uint8_t index)
{

    int ret = -1;
    /*
    uint8_t tmp[3];
    uint16_t start_x,start_y,end_x,end_y,end_y8,start_y8;
    uint8_t cross_tail;
    
    start_x = (x0>x1)?x1:x0;
    start_y = (y0>y1)?y1:y0;
    end_x = x0 + x1 - start_x;
    end_y = y0 + y1 - start_y;
    
    start_y8 = start_y/8;
    end_y = end_y/8;
    
    if((start_y%8 != 0) && (end_y8 != start_y8))
    {
        cross_tail = 1;
    }
    
    if((x0 >= max_x) || (x1 >= max_x) || (y0 >= max_y) || (y1 >= max_y))
    {
        return -1;
    }

    if(dir == UP)
    {
    }
    else if(dir == DOWN)
    {
    }
    else if(dir == LEFT)
    {
    }
    else if(dir == RIGHT)
    {
    }
    */
    return ret;
}

/*************************************************************************                    
*  Description:
*                         AT�����ַ�ƥ�亯��
* Param:   s  Դ�ַ���׵�ַ
* Param:   d  Ŀ���ַ���׵�ַ
* Param:   num �Ƚϵ���󳤶�
* Retval:  0   ƥ��
* Retval   >0  ��ƥ�� 
**************************************************************************/
uint8_t cmdcmp(const char *s,uint8_t *d,uint8_t num)
{
    while(*s++ == *d++)
    {
        if((--num) == 0)
        {
            break;
        }
    }
    return num;
}

/*************************************************************************                    
*  Description:
*                         ��AT�����л�ȡ������������
*���(2,8,5)�л�ȡ���֪���������ֱ�Ϊ2,8,5
* Param:   p  �����ַ���׵�ַ
* Param:   param ����������׵�ַ
* Param:   param_num �������Ĵ�ŵ�ַ
* Retval:  none
**************************************************************************/
void rgb_matrix::get_param(uint8_t *p,uint16_t* param,uint8_t *param_num)
{
    uint16_t tmp= 0;
    uint8_t i,err = 0,num = 0,flag = 0;
    
    
    if(p[0] != '(')
    {
        *param_num = 0xff;
    }
    p++;
    while((*p != ')') && (err==0))
    {
        if((*p >= '0') && (*p <= '9'))
        {
            flag = 1;
            tmp = tmp*10 + (*p - '0');
        }
        else if(*p == ',')
        {
            param[num++] = tmp;
            *param_num = num;
            tmp = 0;
        }
        else
        {
            err = 1;
            *param_num = 0xff;
        }
        p++;
    }
    if(*p == ')')
    {
        if((num != 0) || (flag != 0))
        {
            param[num++] = tmp;
        }
        *param_num = num;
    }
}

/*************************************************************************                    
*  Description:
*                         ִ��ATָ��
* Param:   p  atָ���ŵ��ַ��׵�ַ
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::at_cmd(uint8_t *p)
{
    const char CH[8][15]={"plot","clear","print","put_char","add_layer","remove_layer","move","set_cur"};
    uint8_t i = 0xff,base;
    int16_t ret = 0;
    
    if((p[0] == 'A') && (p[1] = 'T') && (p[2] = '+'))
    {
        for(i = 0; i < 8; i++)
        {
            if(cmdcmp(CH[i],&p[3],strlen(CH[i])) == 0)
            {
                break;
            }
        }
    }
    else
    {
        return -1;
    }
    base = 3 + strlen(CH[i]);

    switch(i)
    {
        case 0:  //plot
            get_param(&p[base],param,&param_num);
            if(param_num == 2)
                plot(param[0],param[1]);
            else if(param_num == 5)
                plot(param[0],param[1],param[2],param[3],param[4]);
            else if(param_num == 4)
                plot(param[0],param[1],param[2],param[3]);
            else if(param_num == 7)
                plot(param[0],param[1],param[2],param[3],param[4],param[5],param[6]);
            else
                ret = -1;
            break;
        case 1:  //clear
            get_param(&p[base],param,&param_num);
            if(param_num == 0)
                clear();
            else if(param_num == 1)
                clear(param[0]);
            else if(param_num == 5)
                clear(param[0],param[1],param[2],param[3],param[4]);
            else
                ret = -1;
            break;
        case 2:  //print
            get_param(&p[base],param,&param_num);
            if(param_num == 2)
                print((uint16_t)param[0],param[1]);
            else if(param_num == 5)
                print((uint16_t)param[0],param[1],param[2],param[3],param[4]);
            else
                ret = -1;
            break;
        case 3:  //put_char
            get_param(&p[base],param,&param_num);
            if(param_num == 6)
                put_char(param[0],param[1],param[2],param[3],param[4],param[5]);
            else if(param_num == 7)
                put_char(param[0],param[1],param[2],param[3],param[4],param[5],param[6]);
            else
                ret = -1;
            break;
        case 4:   //add_layer
            get_param(&p[base],param,&param_num);
            if(param_num == 0)
                add_layer();
            else
                ret = -1;
            break;
        case 5:   //remove_layer
            get_param(&p[base],param,&param_num);
            if(param_num == 0)
                remove_layer();
            else if(param_num == 1)
                remove_layer(param[0]);
            else
                ret = -1;
            break;
        case 6:    // move
            get_param(&p[base],param,&param_num);
            if(param_num == 3)
                move(param[0],param[1],param[2]);
            else
                ret = -1;
            break;
        case 7:    //set_cur
            get_param(&p[base],param,&param_num);
            if(param_num == 1)
                set_cur((uint8_t)param[0]);
            if(param_num == 2)
                set_cur(param[0],param[1]);
            else
                ret = -1;
            break;
        default:
            ret = -1;
            break;
    }
    return ret;
}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ����ȫ����ʾ�ʺ�
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::rainbow(uint8_t index)
{
    uint8_t *p,tmp;
    uint16_t i,j;
    uint8_t x8 = max_x/8, y8 = max_y/8;
    
    if(index >= sum_layer)
    {
        return -1;
    }
    
    struct screens_buffer* screen;
    screen = layer[index];
    p = screen->buffer;
    
    for(j = 0; j < y8; j++)
    {
        p[j] =0x07;
        p[single_color_buffer_size + j] = 0x07<<3;
        p[single_color_buffer_size*2+j] = 0x07<<6;
        /*
        p[j] = (0x07 << (j%8)) | (0x07>>(9-(j%7)));
        p[single_color_buffer_size + j] = (0x07 << ((j+3)%8)) | (0x07>>(9-((j+3)%8)));
        p[single_color_buffer_size*2 +j] = (0x07 << ((j+6)%8)) | (0x07>>(9-((j+6)%8)));
        */
    }
    
    for(i = 1; i< x8*8; i++)
    {
        p[i*y8] = (p[(i-1)*y8] << 1) | (!(!(p[(i+1)*y8 -1 - y8]&0x80)));
        p[single_color_buffer_size+i*y8] = (p[single_color_buffer_size+(i-1)*y8] << 1) | (!(!(p[single_color_buffer_size+(i+1)*y8 -1 - y8]&0x80)));
        p[single_color_buffer_size*2+i*y8] = (p[single_color_buffer_size*2+(i-1)*y8] << 1) | (!(!(p[single_color_buffer_size*2+(i+1)*y8 -1 - y8]&0x80)));
        
        for(j = 1; j < y8; j++)
        {
            p[i*y8 + j] = (p[(i-1)*y8 +j] << 1) | (!(!(p[(i-1)*y8 +j - 1]&0x80)));
            p[single_color_buffer_size+i*y8 + j] = (p[single_color_buffer_size+(i-1)*y8 +j] << 1) | (!(!(p[single_color_buffer_size+(i-1)*y8 +j - 1]&0x80)));
            p[single_color_buffer_size*2+i*y8 + j] = (p[single_color_buffer_size*2+(i-1)*y8 +j] << 1) | (!(!(p[single_color_buffer_size*2+(i-1)*y8 +j - 1]&0x80)));
        }
        
    }
    dirty = 1;
}
/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ����뵥ɫͼƬ
* Param:   x y ��ʾͼƬ����ʼ���
* Param:   size_x size_y ͼƬ�ߴ�(��Ҫ��8�ı���)
* Param:   p ͼƬ���
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_pic(int16_t x, int16_t y, uint8_t size_x, uint8_t size_y, const char p[], uint8_t mode,uint8_t color,uint8_t index)
{
    uint8_t i,j;
    if((x+size_x > max_x) || (y+size_y > max_y))
    {
        return -1;
    }
    
    for(i=0; i<size_x;i++)
    {
        for(j=0; j < size_y/8; j++)
        {
            put_byte(x+i,y+j*8,p[i+size_x*(size_y/8-1)-j*size_x],mode,color,index);
        }
    }

}

/*************************************************************************                    
*  Description:
*                         ��ָ��ͼ����뺺��
* Param:   x y ��ʾͼƬ����ʼ���
* Param:   p ������ģ���
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_HZ(int16_t x, int16_t y, const char p[],uint8_t mode,uint8_t color,uint8_t index)
{
    if((x+16 > max_x) || (y+16 > max_y))
    {
        return -1;
    }
    put_pic(x,y,16,16,p,mode,color,index);
}

/*************************************************************************                    
*  Description:
*                         �򶥲�ͼ����뺺��
* Param:   x y ��ʾͼƬ����ʼ���
* Param:   p ������ģ���
* Param:   mode  MULTIPLY ��ԭ����ݻ��  COVER ����ԭ�����
* Param:   color Ҫд����ݵ���ɫ RED GREEN BLUE������ɫ 
* Param:   index ͼ�������
* Retval:  0   �ɹ�
* Retval   -1  ʧ�� 
**************************************************************************/
int rgb_matrix::put_HZ(int16_t x, int16_t y, const char p[])
{
    if((x+16 > max_x) || (y+16 > max_y))
    {
        return -1;
    }
    put_pic(x,y,16,16,p,COVER,RED,TOP_LAYER);
}

/*void rgb_matrix::dumpStuff(void)
{

}*/
