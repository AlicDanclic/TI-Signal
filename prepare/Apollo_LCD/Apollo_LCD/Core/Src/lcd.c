#include "lcd.h"

/*
 * MCU 并口 LCD 驱动说明：
 * 1. LCD_REG 对应命令寄存器，LCD_RAM 对应数据/显存寄存器；
 * 2. A18 作为 RS，FMC 以 16 位异步 SRAM 方式产生读写时序；
 * 3. LCD_Init 保留正点原子官方控制器识别和初始化序列；
 * 4. 上层绘图统一使用 RGB565 颜色和横屏逻辑坐标。
 */

/*
 * MCU 屏通过 FMC 映射为两个 16 位寄存器：LCD_REG 写命令，LCD_RAM 写像素或参数。
 * 本文件的控制器初始化表已在实物上验证；调整显示界面时优先调用下方绘图 API，
 * 不要随意改变 LCD_Init 中寄存器写入顺序。
 */

static void LCD_DelayUs(uint32_t microseconds)
{
    /* DWT 周期计数器用于初始化阶段的短延时，避免依赖忙等空循环。 */
    uint32_t ticks = (SystemCoreClock / 1000000U) * microseconds;
    uint32_t start;

    /* 使用 Cortex-M4 的 DWT 周期计数器提供初始化阶段需要的微秒延时。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}


u32 POINT_COLOR = 0xFF000000;		
u32 BACK_COLOR = 0xFFFFFFFF;  	



_lcd_dev lcddev;



void LCD_WR_REG(vu16 regval)
{
    /* A18=0：向 LCD 控制器写入命令编号。 */
    /* A18=0：写入控制器命令寄存器。 */
    regval = regval;		
    LCD->LCD_REG = regval; 
}


void LCD_WR_DATA(vu16 data)
{
    /* A18=1：向当前命令写入参数或 RGB565 像素。 */
    /* A18=1：写入命令参数或 RGB565 像素数据。 */
    data = data;			
    LCD->LCD_RAM = data;
}


u16 LCD_RD_DATA(void)
{
    /* 读取 LCD 数据总线，主要用于控制器 ID 和像素读取。 */
    vu16 ram;			
    ram = LCD->LCD_RAM;
    return ram;
}



void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
{
    /* 连续写入一个命令及其单个参数。 */
    LCD->LCD_REG = LCD_Reg;		
    LCD->LCD_RAM = LCD_RegValue;
}



u16 LCD_ReadReg(u16 LCD_Reg)
{
    /* 写入待读寄存器后等待总线稳定，再读取返回值。 */
    LCD_WR_REG(LCD_Reg);		
    LCD_DelayUs(5);
    return LCD_RD_DATA();		
}

void LCD_WriteRAM_Prepare(void)
{
    /* 发送显存写入命令，之后连续写 LCD_RAM 会自动推进 GRAM 地址。 */
    /* 设置 GRAM 写命令后，后续连续写 LCD_RAM 会自动递增显示地址。 */
    LCD->LCD_REG = lcddev.wramcmd;
}


void LCD_WriteRAM(u16 RGB_Code)
{
    LCD->LCD_RAM = RGB_Code;
}




u16 LCD_BGR2RGB(u16 c)
{
    u16  r, g, b, rgb;
    b = (c >> 0) & 0x1f;
    g = (c >> 5) & 0x3f;
    r = (c >> 11) & 0x1f;
    rgb = (b << 11) + (g << 5) + (r << 0);
    return (rgb);
}


void opt_delay(u8 i)
{
    while (i--);
}



u32 LCD_ReadPoint(u16 x, u16 y)
{
    /* 读取指定像素；不同控制器的读显存返回格式略有差异。 */
    u16 r = 0, g = 0, b = 0;

    if (x >= lcddev.width || y >= lcddev.height)return 0;	

    LCD_SetCursor(x, y);

    if (lcddev.id == 0X5510)    
    {
        LCD_WR_REG(0X2E00);
    }
    else                        
    {
        LCD_WR_REG(0X2E);
    }

    r = LCD_RD_DATA();								

    if (lcddev.id == 0X1963)return r;					

    opt_delay(2);
    r = LCD_RD_DATA();  		  						
    if (lcddev.id == 0X7796)    
    {
        return r;
    }
    
    opt_delay(2);
    b = LCD_RD_DATA();
    g = r & 0XFF;		
    g <<= 8;
    return (((r >> 11) << 11) | ((g >> 10) << 5) | (b >> 11));	
}


void LCD_DisplayOn(void)
{
    /* 向控制器发送 Display ON 命令，恢复面板显示。 */
    if (lcddev.id == 0X5510)    
    {
        LCD_WR_REG(0X2900);     
    }
    else                        
    {
        LCD_WR_REG(0X29);       
    }
}

void LCD_DisplayOff(void)
{
    /* 向控制器发送 Display OFF 命令，关闭面板输出但不清除显存。 */
    if (lcddev.id == 0X5510)    
    {
        LCD_WR_REG(0X2800);     
    }
    else                        
    {
        LCD_WR_REG(0X28);       
    }
}



void LCD_SetCursor(u16 Xpos, u16 Ypos)
{
    /* 将逻辑坐标转换为控制器窗口寄存器格式，并设置当前写入位置。 */
    /* 各控制器窗口寄存器格式不同，在这里统一转换为逻辑横屏坐标。 */
    if (lcddev.id == 0X1963)
    {
        if (lcddev.dir == 0)   
        {
            Xpos = lcddev.width - 1 - Xpos;
            LCD_WR_REG(lcddev.setxcmd);
            LCD_WR_DATA(0);
            LCD_WR_DATA(0);
            LCD_WR_DATA(Xpos >> 8);
            LCD_WR_DATA(Xpos & 0XFF);
        }
        else
        {
            LCD_WR_REG(lcddev.setxcmd);
            LCD_WR_DATA(Xpos >> 8);
            LCD_WR_DATA(Xpos & 0XFF);
            LCD_WR_DATA((lcddev.width - 1) >> 8);
            LCD_WR_DATA((lcddev.width - 1) & 0XFF);
        }

        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(Ypos >> 8);
        LCD_WR_DATA(Ypos & 0XFF);
        LCD_WR_DATA((lcddev.height - 1) >> 8);
        LCD_WR_DATA((lcddev.height - 1) & 0XFF);

    }
    else if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(Xpos >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(Xpos & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(Ypos >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(Ypos & 0XFF);
    }
    else     
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(Xpos >> 8);
        LCD_WR_DATA(Xpos & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(Ypos >> 8);
        LCD_WR_DATA(Ypos & 0XFF);
    }
}





void LCD_Scan_Dir(u8 dir)
{
    /* 设置扫描方向，同时更新驱动层记录的宽高，保证绘图坐标一致。 */
    u16 regval = 0;
    u16 dirreg = 0;
    u16 temp;

    
    if ((lcddev.dir == 1 && lcddev.id != 0X1963) || (lcddev.dir == 0 && lcddev.id == 0X1963)) 
    {
        switch (dir) 
        {
            case 0:
                dir = 6;
                break;

            case 1:
                dir = 7;
                break;

            case 2:
                dir = 4;
                break;

            case 3:
                dir = 5;
                break;

            case 4:
                dir = 1;
                break;

            case 5:
                dir = 0;
                break;

            case 6:
                dir = 3;
                break;

            case 7:
                dir = 2;
                break;
        }
    }

    switch (dir)
    {
        case L2R_U2D:
            regval |= (0 << 7) | (0 << 6) | (0 << 5);
            break;

        case L2R_D2U:
            regval |= (1 << 7) | (0 << 6) | (0 << 5);
            break;

        case R2L_U2D:
            regval |= (0 << 7) | (1 << 6) | (0 << 5);
            break;

        case R2L_D2U:
            regval |= (1 << 7) | (1 << 6) | (0 << 5);
            break;

        case U2D_L2R:
            regval |= (0 << 7) | (0 << 6) | (1 << 5);
            break;

        case U2D_R2L:
            regval |= (0 << 7) | (1 << 6) | (1 << 5);
            break;

        case D2U_L2R:
            regval |= (1 << 7) | (0 << 6) | (1 << 5);
            break;

        case D2U_R2L:
            regval |= (1 << 7) | (1 << 6) | (1 << 5);
            break;
    }
    dirreg = 0X36;

    if (lcddev.id == 0X5510)dirreg = 0X3600;

    if (lcddev.id == 0X9341 || lcddev.id == 0X7789 || lcddev.id == 0x7796)   
    {
        regval |= 0X08;
    }

    LCD_WriteReg(dirreg, regval);

    if (lcddev.id != 0X1963) 
    {
        if (regval & 0X20)
        {
            if (lcddev.width < lcddev.height) 
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
        else
        {
            if (lcddev.width > lcddev.height) 
            {
                temp = lcddev.width;
                lcddev.width = lcddev.height;
                lcddev.height = temp;
            }
        }
    }

    if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setxcmd + 2);
        LCD_WR_DATA((lcddev.width - 1) >> 8);
        LCD_WR_REG(lcddev.setxcmd + 3);
        LCD_WR_DATA((lcddev.width - 1) & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(0);
        LCD_WR_REG(lcddev.setycmd + 2);
        LCD_WR_DATA((lcddev.height - 1) >> 8);
        LCD_WR_REG(lcddev.setycmd + 3);
        LCD_WR_DATA((lcddev.height - 1) & 0XFF);
    }
    else
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);
        LCD_WR_DATA((lcddev.width - 1) >> 8);
        LCD_WR_DATA((lcddev.width - 1) & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(0);
        LCD_WR_DATA(0);
        LCD_WR_DATA((lcddev.height - 1) >> 8);
        LCD_WR_DATA((lcddev.height - 1) & 0XFF);
    }
}



void LCD_DrawPoint(u16 x, u16 y)
{
    /* 在单个坐标写入当前画笔颜色 POINT_COLOR。 */
    LCD_SetCursor(x, y);		
    LCD_WriteRAM_Prepare();	
    LCD->LCD_RAM = POINT_COLOR;
}



void LCD_Fast_DrawPoint(u16 x, u16 y, u32 color)
{
    /* 省略部分通用检查的快速单点绘制，适合波形刷新路径。 */
    if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(y >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(y & 0XFF);
    }
    else if (lcddev.id == 0X1963)
    {
        if (lcddev.dir == 0)x = lcddev.width - 1 - x;

        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x >> 8);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_DATA(x >> 8);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(y >> 8);
        LCD_WR_DATA(y & 0XFF);
        LCD_WR_DATA(y >> 8);
        LCD_WR_DATA(y & 0XFF);
    }
    else     
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(x >> 8);
        LCD_WR_DATA(x & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(y >> 8);
        LCD_WR_DATA(y & 0XFF);
    }

    LCD->LCD_REG=lcddev.wramcmd; 
    LCD->LCD_RAM=color; 
}


void LCD_SSD_BackLightSet(u8 pwm)
{
    /* SSD1963 背光 PWM 接口；当前模块主要使用 PB5 硬件背光使能。 */
    LCD_WR_REG(0xBE);	
    LCD_WR_DATA(0x05);	
    LCD_WR_DATA(pwm * 2.55); 
    LCD_WR_DATA(0x01);	
    LCD_WR_DATA(0xFF);	
    LCD_WR_DATA(0x00);	
    LCD_WR_DATA(0x00);	
}



void LCD_Display_Dir(u8 dir)
{
    /* 选择横屏/竖屏显示方向，并同步设置控制器扫描寄存器。 */
     lcddev.dir = dir;       

    if (dir == 0)           
    {
        lcddev.width = 240;
        lcddev.height = 320;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 480;
            lcddev.height = 800;
        }
        else if (lcddev.id == 0X1963)
        {
            lcddev.wramcmd = 0X2C;  
            lcddev.setxcmd = 0X2B;  
            lcddev.setycmd = 0X2A;  
            lcddev.width = 480;     
            lcddev.height = 800;    
        }
        else                        
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310 || lcddev.id == 0x7796)    
        {
            lcddev.width = 320;
            lcddev.height = 480;
        }
        if (lcddev.id == 0X9806)    
        {
            lcddev.width = 480;
            lcddev.height = 800;
        }  
    }
    else     
    {
        lcddev.width = 320;
        lcddev.height = 240;

        if (lcddev.id == 0x5510)
        {
            lcddev.wramcmd = 0X2C00;
            lcddev.setxcmd = 0X2A00;
            lcddev.setycmd = 0X2B00;
            lcddev.width = 800;
            lcddev.height = 480;
        }
        else if (lcddev.id == 0X1963 || lcddev.id == 0x9806)
        {
            lcddev.wramcmd = 0X2C;  
            lcddev.setxcmd = 0X2A;  
            lcddev.setycmd = 0X2B;  
            lcddev.width = 800;     
            lcddev.height = 480;    
        }
        else                        
        {
            lcddev.wramcmd = 0X2C;
            lcddev.setxcmd = 0X2A;
            lcddev.setycmd = 0X2B;
        }

        if (lcddev.id == 0X5310 || lcddev.id == 0x7796)    
        {
            lcddev.width = 480;
            lcddev.height = 320;
        }
    }

    LCD_Scan_Dir(DFT_SCAN_DIR);     
}





void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height)
{
    /* 设置矩形显存窗口，后续连续写 RAM 可批量填充该区域。 */
    /* 将左上角和宽高转换为控制器所需的起止坐标。 */
    u16 twidth, theight;
    twidth = sx + width - 1;
    theight = sy + height - 1;

    if (lcddev.id == 0X1963 && lcddev.dir != 1)     
    {
        sx = lcddev.width - width - sx;
        height = sy + height - 1;
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(sx >> 8);
        LCD_WR_DATA(sx & 0XFF);
        LCD_WR_DATA((sx + width - 1) >> 8);
        LCD_WR_DATA((sx + width - 1) & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(sy >> 8);
        LCD_WR_DATA(sy & 0XFF);
        LCD_WR_DATA(height >> 8);
        LCD_WR_DATA(height & 0XFF);
    }
    else if (lcddev.id == 0X5510)
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(sx >> 8);
        LCD_WR_REG(lcddev.setxcmd + 1);
        LCD_WR_DATA(sx & 0XFF);
        LCD_WR_REG(lcddev.setxcmd + 2);
        LCD_WR_DATA(twidth >> 8);
        LCD_WR_REG(lcddev.setxcmd + 3);
        LCD_WR_DATA(twidth & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(sy >> 8);
        LCD_WR_REG(lcddev.setycmd + 1);
        LCD_WR_DATA(sy & 0XFF);
        LCD_WR_REG(lcddev.setycmd + 2);
        LCD_WR_DATA(theight >> 8);
        LCD_WR_REG(lcddev.setycmd + 3);
        LCD_WR_DATA(theight & 0XFF);
    }
    else     
    {
        LCD_WR_REG(lcddev.setxcmd);
        LCD_WR_DATA(sx >> 8);
        LCD_WR_DATA(sx & 0XFF);
        LCD_WR_DATA(twidth >> 8);
        LCD_WR_DATA(twidth & 0XFF);
        LCD_WR_REG(lcddev.setycmd);
        LCD_WR_DATA(sy >> 8);
        LCD_WR_DATA(sy & 0XFF);
        LCD_WR_DATA(theight >> 8);
        LCD_WR_DATA(theight & 0XFF);
    }
}



void LCD_Init(void)
{
    /* 读取多个候选 ID，随后执行对应控制器的官方上电初始化序列。 */
    /* 控制器上电稳定时间结束后，依次尝试读取各兼容控制器 ID。 */
    HAL_Delay(50);

    
    LCD_WR_REG(0XD3);
    lcddev.id = LCD_RD_DATA();	
    lcddev.id = LCD_RD_DATA();	
    lcddev.id = LCD_RD_DATA();   	
    lcddev.id <<= 8;
    lcddev.id |= LCD_RD_DATA();  	

    if (lcddev.id != 0X9341)		
    {
        LCD_WR_REG(0X04);
        lcddev.id = LCD_RD_DATA();      
        lcddev.id = LCD_RD_DATA();      
        lcddev.id = LCD_RD_DATA();      
        lcddev.id <<= 8;
        lcddev.id |= LCD_RD_DATA();     

        if (lcddev.id == 0X8552)        
        {
            lcddev.id = 0x7789;
        }

        if (lcddev.id != 0x7789)        
        {
            LCD_WR_REG(0XD4);
            lcddev.id = LCD_RD_DATA(); 
            lcddev.id = LCD_RD_DATA(); 
            lcddev.id = LCD_RD_DATA(); 
            lcddev.id <<= 8;
            lcddev.id |= LCD_RD_DATA();	

            if (lcddev.id != 0X5310)    
            {
                LCD_WR_REG(0XD3);
                lcddev.id = LCD_RD_DATA();  /* dummy read */
                lcddev.id = LCD_RD_DATA();  /* 0X00 */
                lcddev.id = LCD_RD_DATA();  /* 0X77 */
                lcddev.id <<= 8;
                lcddev.id |= LCD_RD_DATA(); /* 0X96 */
                
                if (lcddev.id != 0x7796)    /* ST7796,NT35510 */
                {
                    
                    LCD_WriteReg(0xF000, 0x0055);
                    LCD_WriteReg(0xF001, 0x00AA);
                    LCD_WriteReg(0xF002, 0x0052);
                    LCD_WriteReg(0xF003, 0x0008);
                    LCD_WriteReg(0xF004, 0x0001);

                    LCD_WR_REG(0xC500);             
                    lcddev.id = LCD_RD_DATA();      
                    lcddev.id <<= 8;

                    LCD_WR_REG(0xC501);             
                    lcddev.id |= LCD_RD_DATA();     
                    HAL_Delay(5);

                    if (lcddev.id != 0X5510)        
                    {
                        LCD_WR_REG(0XD3);
                        lcddev.id = LCD_RD_DATA();  /* dummy read */
                        lcddev.id = LCD_RD_DATA();  /* 0X00 */
                        lcddev.id = LCD_RD_DATA();  /* 0X98 */
                        lcddev.id <<= 8;
                        lcddev.id |= LCD_RD_DATA(); /* 0X06 */
                        
                        if (lcddev.id != 0x9806)    /* ILI9806,SSD1963 */
                        {
                            LCD_WR_REG(0XA1);
                            lcddev.id = LCD_RD_DATA();
                            lcddev.id = LCD_RD_DATA();  
                            lcddev.id <<= 8;
                            lcddev.id |= LCD_RD_DATA(); 

                            if (lcddev.id == 0X5761)lcddev.id = 0X1963; 
                        }
                    }
                }
            }
        }
    }

    if (lcddev.id == 0X9341)	
    {
        LCD_WR_REG(0xCF);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC1);
        LCD_WR_DATA(0X30);
        LCD_WR_REG(0xED);
        LCD_WR_DATA(0x64);
        LCD_WR_DATA(0x03);
        LCD_WR_DATA(0X12);
        LCD_WR_DATA(0X81);
        LCD_WR_REG(0xE8);
        LCD_WR_DATA(0x85);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x7A);
        LCD_WR_REG(0xCB);
        LCD_WR_DATA(0x39);
        LCD_WR_DATA(0x2C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x34);
        LCD_WR_DATA(0x02);
        LCD_WR_REG(0xF7);
        LCD_WR_DATA(0x20);
        LCD_WR_REG(0xEA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0xC0);    
        LCD_WR_DATA(0x1B);   
        LCD_WR_REG(0xC1);    
        LCD_WR_DATA(0x01);   
        LCD_WR_REG(0xC5);    
        LCD_WR_DATA(0x30); 	 
        LCD_WR_DATA(0x30); 	 
        LCD_WR_REG(0xC7);    
        LCD_WR_DATA(0XB7);
        LCD_WR_REG(0x36);    
        LCD_WR_DATA(0x48);
        LCD_WR_REG(0x3A);
        LCD_WR_DATA(0x55);
        LCD_WR_REG(0xB1);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x1A);
        LCD_WR_REG(0xB6);    
        LCD_WR_DATA(0x0A);
        LCD_WR_DATA(0xA2);
        LCD_WR_REG(0xF2);    
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0x26);    
        LCD_WR_DATA(0x01);
        LCD_WR_REG(0xE0);    
        LCD_WR_DATA(0x0F);
        LCD_WR_DATA(0x2A);
        LCD_WR_DATA(0x28);
        LCD_WR_DATA(0x08);
        LCD_WR_DATA(0x0E);
        LCD_WR_DATA(0x08);
        LCD_WR_DATA(0x54);
        LCD_WR_DATA(0XA9);
        LCD_WR_DATA(0x43);
        LCD_WR_DATA(0x0A);
        LCD_WR_DATA(0x0F);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0XE1);    
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x15);
        LCD_WR_DATA(0x17);
        LCD_WR_DATA(0x07);
        LCD_WR_DATA(0x11);
        LCD_WR_DATA(0x06);
        LCD_WR_DATA(0x2B);
        LCD_WR_DATA(0x56);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x05);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x0F);
        LCD_WR_DATA(0x3F);
        LCD_WR_DATA(0x3F);
        LCD_WR_DATA(0x0F);
        LCD_WR_REG(0x2B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x3f);
        LCD_WR_REG(0x2A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xef);
        LCD_WR_REG(0x11); 
        HAL_Delay(120);
        LCD_WR_REG(0x29); 
    }
    else if (lcddev.id == 0x7789)   
    {
        LCD_WR_REG(0x11);

        HAL_Delay(120);

        LCD_WR_REG(0x36);
        LCD_WR_DATA(0x00);


        LCD_WR_REG(0x3A);
        LCD_WR_DATA(0X05);

        LCD_WR_REG(0xB2);
        LCD_WR_DATA(0x0C);
        LCD_WR_DATA(0x0C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x33);
        LCD_WR_DATA(0x33);

        LCD_WR_REG(0xB7);
        LCD_WR_DATA(0x35);

        LCD_WR_REG(0xBB);       
        LCD_WR_DATA(0x32);      

        LCD_WR_REG(0xC0);
        LCD_WR_DATA(0x0C);

        LCD_WR_REG(0xC2);
        LCD_WR_DATA(0x01);

        LCD_WR_REG(0xC3);       
        LCD_WR_DATA(0x10);      

        LCD_WR_REG(0xC4);       
        LCD_WR_DATA(0x20);      

        LCD_WR_REG(0xC6);
        LCD_WR_DATA(0x0f);

        LCD_WR_REG(0xD0);
        LCD_WR_DATA(0xA4);
        LCD_WR_DATA(0xA1);

        LCD_WR_REG(0xE0);       
        LCD_WR_DATA(0xd0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x07);
        LCD_WR_DATA(0x0a);
        LCD_WR_DATA(0x28);
        LCD_WR_DATA(0x32);
        LCD_WR_DATA(0X44);
        LCD_WR_DATA(0x42);
        LCD_WR_DATA(0x06);
        LCD_WR_DATA(0x0e);
        LCD_WR_DATA(0x12);
        LCD_WR_DATA(0x14);
        LCD_WR_DATA(0x17);


        LCD_WR_REG(0XE1);       
        LCD_WR_DATA(0xd0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x07);
        LCD_WR_DATA(0x0a);
        LCD_WR_DATA(0x28);
        LCD_WR_DATA(0x31);
        LCD_WR_DATA(0x54);
        LCD_WR_DATA(0x47);
        LCD_WR_DATA(0x0e);
        LCD_WR_DATA(0x1c);
        LCD_WR_DATA(0x17);
        LCD_WR_DATA(0x1b);
        LCD_WR_DATA(0x1e);

        LCD_WR_REG(0x2A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xef);

        LCD_WR_REG(0x2B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x3f);

        LCD_WR_REG(0x29);       
    }
    else if (lcddev.id == 0x5310)
    {
        LCD_WR_REG(0xED);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0xFE);

        LCD_WR_REG(0xEE);
        LCD_WR_DATA(0xDE);
        LCD_WR_DATA(0x21);

        LCD_WR_REG(0xF1);
        LCD_WR_DATA(0x01);
        LCD_WR_REG(0xDF);
        LCD_WR_DATA(0x10);

        
        LCD_WR_REG(0xC4);
        LCD_WR_DATA(0x8F);	  

        LCD_WR_REG(0xC6);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xE2);
        LCD_WR_DATA(0xE2);
        LCD_WR_DATA(0xE2);
        LCD_WR_REG(0xBF);
        LCD_WR_DATA(0xAA);

        LCD_WR_REG(0xB0);
        LCD_WR_DATA(0x0D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x0D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x11);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x19);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x21);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x5D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x5D);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB1);
        LCD_WR_DATA(0x80);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x8B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x96);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x03);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB4);
        LCD_WR_DATA(0x8B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x96);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA1);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB5);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x03);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x04);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB6);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3F);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x5E);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x64);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x8C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xAC);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDC);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x70);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x90);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xEB);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDC);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xB8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xBA);
        LCD_WR_DATA(0x24);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC1);
        LCD_WR_DATA(0x20);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x54);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC2);
        LCD_WR_DATA(0x0A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x04);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC3);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x39);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x37);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x36);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x32);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2F);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x29);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x26);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x24);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x24);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x23);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x36);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x32);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2F);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x29);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x26);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x24);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x24);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x23);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC4);
        LCD_WR_DATA(0x62);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x05);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x84);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x18);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA4);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x18);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x50);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x0C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x17);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x95);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xE6);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC5);
        LCD_WR_DATA(0x32);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x65);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x76);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC6);
        LCD_WR_DATA(0x20);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x17);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xC9);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE0);
        LCD_WR_DATA(0x16);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x1C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x21);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x36);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x46);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x52);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x64);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x7A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x8B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB9);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC4);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xCA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD9);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xE0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE1);
        LCD_WR_DATA(0x16);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x1C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x22);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x36);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x45);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x52);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x64);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x7A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x8B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB9);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC4);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xCA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xE0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE2);
        LCD_WR_DATA(0x05);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x0B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x1B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x34);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x4F);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x61);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x79);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x97);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA6);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD1);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD6);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDD);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0xE3);
        LCD_WR_DATA(0x05);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x1C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x33);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x50);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x62);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x78);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x97);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA6);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC7);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD1);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD5);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDD);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE4);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x2A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x4B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x5D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x74);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x84);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x93);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xBE);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC4);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xCD);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDD);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0xE5);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x29);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x3C);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x4B);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x5D);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x74);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x84);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x93);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xA2);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xB3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xBE);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xC4);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xCD);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xD3);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xDC);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xF3);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE6);
        LCD_WR_DATA(0x11);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x34);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x56);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x76);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x77);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x66);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xBB);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x66);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x45);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x43);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE7);
        LCD_WR_DATA(0x32);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x76);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x66);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x67);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x67);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x87);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xBB);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x77);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x56);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x23);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x33);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x45);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE8);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x87);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x77);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x66);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x88);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xAA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xBB);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x99);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x66);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x55);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xE9);
        LCD_WR_DATA(0xAA);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0x00);
        LCD_WR_DATA(0xAA);

        LCD_WR_REG(0xCF);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xF0);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x50);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xF3);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xF9);
        LCD_WR_DATA(0x06);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x29);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0x3A);
        LCD_WR_DATA(0x55);	

        LCD_WR_REG(0x11);
        HAL_Delay(100);
        LCD_WR_REG(0x29);
        LCD_WR_REG(0x35);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0x51);
        LCD_WR_DATA(0xFF);
        LCD_WR_REG(0x53);
        LCD_WR_DATA(0x2C);
        LCD_WR_REG(0x55);
        LCD_WR_DATA(0x82);
        LCD_WR_REG(0x2c);
    }
    else if (lcddev.id == 0x5510)
    {
        LCD_WriteReg(0xF000, 0x55);
        LCD_WriteReg(0xF001, 0xAA);
        LCD_WriteReg(0xF002, 0x52);
        LCD_WriteReg(0xF003, 0x08);
        LCD_WriteReg(0xF004, 0x01);
        
        LCD_WriteReg(0xB000, 0x0D);
        LCD_WriteReg(0xB001, 0x0D);
        LCD_WriteReg(0xB002, 0x0D);
        
        LCD_WriteReg(0xB600, 0x34);
        LCD_WriteReg(0xB601, 0x34);
        LCD_WriteReg(0xB602, 0x34);
        
        LCD_WriteReg(0xB100, 0x0D);
        LCD_WriteReg(0xB101, 0x0D);
        LCD_WriteReg(0xB102, 0x0D);
        
        LCD_WriteReg(0xB700, 0x34);
        LCD_WriteReg(0xB701, 0x34);
        LCD_WriteReg(0xB702, 0x34);
        
        LCD_WriteReg(0xB200, 0x00);
        LCD_WriteReg(0xB201, 0x00);
        LCD_WriteReg(0xB202, 0x00);
        
        LCD_WriteReg(0xB800, 0x24);
        LCD_WriteReg(0xB801, 0x24);
        LCD_WriteReg(0xB802, 0x24);
        
        LCD_WriteReg(0xBF00, 0x01);
        LCD_WriteReg(0xB300, 0x0F);
        LCD_WriteReg(0xB301, 0x0F);
        LCD_WriteReg(0xB302, 0x0F);
        
        LCD_WriteReg(0xB900, 0x34);
        LCD_WriteReg(0xB901, 0x34);
        LCD_WriteReg(0xB902, 0x34);
        
        LCD_WriteReg(0xB500, 0x08);
        LCD_WriteReg(0xB501, 0x08);
        LCD_WriteReg(0xB502, 0x08);
        LCD_WriteReg(0xC200, 0x03);
        
        LCD_WriteReg(0xBA00, 0x24);
        LCD_WriteReg(0xBA01, 0x24);
        LCD_WriteReg(0xBA02, 0x24);
        
        LCD_WriteReg(0xBC00, 0x00);
        LCD_WriteReg(0xBC01, 0x78);
        LCD_WriteReg(0xBC02, 0x00);
        
        LCD_WriteReg(0xBD00, 0x00);
        LCD_WriteReg(0xBD01, 0x78);
        LCD_WriteReg(0xBD02, 0x00);
        
        LCD_WriteReg(0xBE00, 0x00);
        LCD_WriteReg(0xBE01, 0x64);
        
        LCD_WriteReg(0xD100, 0x00);
        LCD_WriteReg(0xD101, 0x33);
        LCD_WriteReg(0xD102, 0x00);
        LCD_WriteReg(0xD103, 0x34);
        LCD_WriteReg(0xD104, 0x00);
        LCD_WriteReg(0xD105, 0x3A);
        LCD_WriteReg(0xD106, 0x00);
        LCD_WriteReg(0xD107, 0x4A);
        LCD_WriteReg(0xD108, 0x00);
        LCD_WriteReg(0xD109, 0x5C);
        LCD_WriteReg(0xD10A, 0x00);
        LCD_WriteReg(0xD10B, 0x81);
        LCD_WriteReg(0xD10C, 0x00);
        LCD_WriteReg(0xD10D, 0xA6);
        LCD_WriteReg(0xD10E, 0x00);
        LCD_WriteReg(0xD10F, 0xE5);
        LCD_WriteReg(0xD110, 0x01);
        LCD_WriteReg(0xD111, 0x13);
        LCD_WriteReg(0xD112, 0x01);
        LCD_WriteReg(0xD113, 0x54);
        LCD_WriteReg(0xD114, 0x01);
        LCD_WriteReg(0xD115, 0x82);
        LCD_WriteReg(0xD116, 0x01);
        LCD_WriteReg(0xD117, 0xCA);
        LCD_WriteReg(0xD118, 0x02);
        LCD_WriteReg(0xD119, 0x00);
        LCD_WriteReg(0xD11A, 0x02);
        LCD_WriteReg(0xD11B, 0x01);
        LCD_WriteReg(0xD11C, 0x02);
        LCD_WriteReg(0xD11D, 0x34);
        LCD_WriteReg(0xD11E, 0x02);
        LCD_WriteReg(0xD11F, 0x67);
        LCD_WriteReg(0xD120, 0x02);
        LCD_WriteReg(0xD121, 0x84);
        LCD_WriteReg(0xD122, 0x02);
        LCD_WriteReg(0xD123, 0xA4);
        LCD_WriteReg(0xD124, 0x02);
        LCD_WriteReg(0xD125, 0xB7);
        LCD_WriteReg(0xD126, 0x02);
        LCD_WriteReg(0xD127, 0xCF);
        LCD_WriteReg(0xD128, 0x02);
        LCD_WriteReg(0xD129, 0xDE);
        LCD_WriteReg(0xD12A, 0x02);
        LCD_WriteReg(0xD12B, 0xF2);
        LCD_WriteReg(0xD12C, 0x02);
        LCD_WriteReg(0xD12D, 0xFE);
        LCD_WriteReg(0xD12E, 0x03);
        LCD_WriteReg(0xD12F, 0x10);
        LCD_WriteReg(0xD130, 0x03);
        LCD_WriteReg(0xD131, 0x33);
        LCD_WriteReg(0xD132, 0x03);
        LCD_WriteReg(0xD133, 0x6D);
        LCD_WriteReg(0xD200, 0x00);
        LCD_WriteReg(0xD201, 0x33);
        LCD_WriteReg(0xD202, 0x00);
        LCD_WriteReg(0xD203, 0x34);
        LCD_WriteReg(0xD204, 0x00);
        LCD_WriteReg(0xD205, 0x3A);
        LCD_WriteReg(0xD206, 0x00);
        LCD_WriteReg(0xD207, 0x4A);
        LCD_WriteReg(0xD208, 0x00);
        LCD_WriteReg(0xD209, 0x5C);
        LCD_WriteReg(0xD20A, 0x00);

        LCD_WriteReg(0xD20B, 0x81);
        LCD_WriteReg(0xD20C, 0x00);
        LCD_WriteReg(0xD20D, 0xA6);
        LCD_WriteReg(0xD20E, 0x00);
        LCD_WriteReg(0xD20F, 0xE5);
        LCD_WriteReg(0xD210, 0x01);
        LCD_WriteReg(0xD211, 0x13);
        LCD_WriteReg(0xD212, 0x01);
        LCD_WriteReg(0xD213, 0x54);
        LCD_WriteReg(0xD214, 0x01);
        LCD_WriteReg(0xD215, 0x82);
        LCD_WriteReg(0xD216, 0x01);
        LCD_WriteReg(0xD217, 0xCA);
        LCD_WriteReg(0xD218, 0x02);
        LCD_WriteReg(0xD219, 0x00);
        LCD_WriteReg(0xD21A, 0x02);
        LCD_WriteReg(0xD21B, 0x01);
        LCD_WriteReg(0xD21C, 0x02);
        LCD_WriteReg(0xD21D, 0x34);
        LCD_WriteReg(0xD21E, 0x02);
        LCD_WriteReg(0xD21F, 0x67);
        LCD_WriteReg(0xD220, 0x02);
        LCD_WriteReg(0xD221, 0x84);
        LCD_WriteReg(0xD222, 0x02);
        LCD_WriteReg(0xD223, 0xA4);
        LCD_WriteReg(0xD224, 0x02);
        LCD_WriteReg(0xD225, 0xB7);
        LCD_WriteReg(0xD226, 0x02);
        LCD_WriteReg(0xD227, 0xCF);
        LCD_WriteReg(0xD228, 0x02);
        LCD_WriteReg(0xD229, 0xDE);
        LCD_WriteReg(0xD22A, 0x02);
        LCD_WriteReg(0xD22B, 0xF2);
        LCD_WriteReg(0xD22C, 0x02);
        LCD_WriteReg(0xD22D, 0xFE);
        LCD_WriteReg(0xD22E, 0x03);
        LCD_WriteReg(0xD22F, 0x10);
        LCD_WriteReg(0xD230, 0x03);
        LCD_WriteReg(0xD231, 0x33);
        LCD_WriteReg(0xD232, 0x03);
        LCD_WriteReg(0xD233, 0x6D);
        LCD_WriteReg(0xD300, 0x00);
        LCD_WriteReg(0xD301, 0x33);
        LCD_WriteReg(0xD302, 0x00);
        LCD_WriteReg(0xD303, 0x34);
        LCD_WriteReg(0xD304, 0x00);
        LCD_WriteReg(0xD305, 0x3A);
        LCD_WriteReg(0xD306, 0x00);
        LCD_WriteReg(0xD307, 0x4A);
        LCD_WriteReg(0xD308, 0x00);
        LCD_WriteReg(0xD309, 0x5C);
        LCD_WriteReg(0xD30A, 0x00);

        LCD_WriteReg(0xD30B, 0x81);
        LCD_WriteReg(0xD30C, 0x00);
        LCD_WriteReg(0xD30D, 0xA6);
        LCD_WriteReg(0xD30E, 0x00);
        LCD_WriteReg(0xD30F, 0xE5);
        LCD_WriteReg(0xD310, 0x01);
        LCD_WriteReg(0xD311, 0x13);
        LCD_WriteReg(0xD312, 0x01);
        LCD_WriteReg(0xD313, 0x54);
        LCD_WriteReg(0xD314, 0x01);
        LCD_WriteReg(0xD315, 0x82);
        LCD_WriteReg(0xD316, 0x01);
        LCD_WriteReg(0xD317, 0xCA);
        LCD_WriteReg(0xD318, 0x02);
        LCD_WriteReg(0xD319, 0x00);
        LCD_WriteReg(0xD31A, 0x02);
        LCD_WriteReg(0xD31B, 0x01);
        LCD_WriteReg(0xD31C, 0x02);
        LCD_WriteReg(0xD31D, 0x34);
        LCD_WriteReg(0xD31E, 0x02);
        LCD_WriteReg(0xD31F, 0x67);
        LCD_WriteReg(0xD320, 0x02);
        LCD_WriteReg(0xD321, 0x84);
        LCD_WriteReg(0xD322, 0x02);
        LCD_WriteReg(0xD323, 0xA4);
        LCD_WriteReg(0xD324, 0x02);
        LCD_WriteReg(0xD325, 0xB7);
        LCD_WriteReg(0xD326, 0x02);
        LCD_WriteReg(0xD327, 0xCF);
        LCD_WriteReg(0xD328, 0x02);
        LCD_WriteReg(0xD329, 0xDE);
        LCD_WriteReg(0xD32A, 0x02);
        LCD_WriteReg(0xD32B, 0xF2);
        LCD_WriteReg(0xD32C, 0x02);
        LCD_WriteReg(0xD32D, 0xFE);
        LCD_WriteReg(0xD32E, 0x03);
        LCD_WriteReg(0xD32F, 0x10);
        LCD_WriteReg(0xD330, 0x03);
        LCD_WriteReg(0xD331, 0x33);
        LCD_WriteReg(0xD332, 0x03);
        LCD_WriteReg(0xD333, 0x6D);
        LCD_WriteReg(0xD400, 0x00);
        LCD_WriteReg(0xD401, 0x33);
        LCD_WriteReg(0xD402, 0x00);
        LCD_WriteReg(0xD403, 0x34);
        LCD_WriteReg(0xD404, 0x00);
        LCD_WriteReg(0xD405, 0x3A);
        LCD_WriteReg(0xD406, 0x00);
        LCD_WriteReg(0xD407, 0x4A);
        LCD_WriteReg(0xD408, 0x00);
        LCD_WriteReg(0xD409, 0x5C);
        LCD_WriteReg(0xD40A, 0x00);
        LCD_WriteReg(0xD40B, 0x81);

        LCD_WriteReg(0xD40C, 0x00);
        LCD_WriteReg(0xD40D, 0xA6);
        LCD_WriteReg(0xD40E, 0x00);
        LCD_WriteReg(0xD40F, 0xE5);
        LCD_WriteReg(0xD410, 0x01);
        LCD_WriteReg(0xD411, 0x13);
        LCD_WriteReg(0xD412, 0x01);
        LCD_WriteReg(0xD413, 0x54);
        LCD_WriteReg(0xD414, 0x01);
        LCD_WriteReg(0xD415, 0x82);
        LCD_WriteReg(0xD416, 0x01);
        LCD_WriteReg(0xD417, 0xCA);
        LCD_WriteReg(0xD418, 0x02);
        LCD_WriteReg(0xD419, 0x00);
        LCD_WriteReg(0xD41A, 0x02);
        LCD_WriteReg(0xD41B, 0x01);
        LCD_WriteReg(0xD41C, 0x02);
        LCD_WriteReg(0xD41D, 0x34);
        LCD_WriteReg(0xD41E, 0x02);
        LCD_WriteReg(0xD41F, 0x67);
        LCD_WriteReg(0xD420, 0x02);
        LCD_WriteReg(0xD421, 0x84);
        LCD_WriteReg(0xD422, 0x02);
        LCD_WriteReg(0xD423, 0xA4);
        LCD_WriteReg(0xD424, 0x02);
        LCD_WriteReg(0xD425, 0xB7);
        LCD_WriteReg(0xD426, 0x02);
        LCD_WriteReg(0xD427, 0xCF);
        LCD_WriteReg(0xD428, 0x02);
        LCD_WriteReg(0xD429, 0xDE);
        LCD_WriteReg(0xD42A, 0x02);
        LCD_WriteReg(0xD42B, 0xF2);
        LCD_WriteReg(0xD42C, 0x02);
        LCD_WriteReg(0xD42D, 0xFE);
        LCD_WriteReg(0xD42E, 0x03);
        LCD_WriteReg(0xD42F, 0x10);
        LCD_WriteReg(0xD430, 0x03);
        LCD_WriteReg(0xD431, 0x33);
        LCD_WriteReg(0xD432, 0x03);
        LCD_WriteReg(0xD433, 0x6D);
        LCD_WriteReg(0xD500, 0x00);
        LCD_WriteReg(0xD501, 0x33);
        LCD_WriteReg(0xD502, 0x00);
        LCD_WriteReg(0xD503, 0x34);
        LCD_WriteReg(0xD504, 0x00);
        LCD_WriteReg(0xD505, 0x3A);
        LCD_WriteReg(0xD506, 0x00);
        LCD_WriteReg(0xD507, 0x4A);
        LCD_WriteReg(0xD508, 0x00);
        LCD_WriteReg(0xD509, 0x5C);
        LCD_WriteReg(0xD50A, 0x00);
        LCD_WriteReg(0xD50B, 0x81);

        LCD_WriteReg(0xD50C, 0x00);
        LCD_WriteReg(0xD50D, 0xA6);
        LCD_WriteReg(0xD50E, 0x00);
        LCD_WriteReg(0xD50F, 0xE5);
        LCD_WriteReg(0xD510, 0x01);
        LCD_WriteReg(0xD511, 0x13);
        LCD_WriteReg(0xD512, 0x01);
        LCD_WriteReg(0xD513, 0x54);
        LCD_WriteReg(0xD514, 0x01);
        LCD_WriteReg(0xD515, 0x82);
        LCD_WriteReg(0xD516, 0x01);
        LCD_WriteReg(0xD517, 0xCA);
        LCD_WriteReg(0xD518, 0x02);
        LCD_WriteReg(0xD519, 0x00);
        LCD_WriteReg(0xD51A, 0x02);
        LCD_WriteReg(0xD51B, 0x01);
        LCD_WriteReg(0xD51C, 0x02);
        LCD_WriteReg(0xD51D, 0x34);
        LCD_WriteReg(0xD51E, 0x02);
        LCD_WriteReg(0xD51F, 0x67);
        LCD_WriteReg(0xD520, 0x02);
        LCD_WriteReg(0xD521, 0x84);
        LCD_WriteReg(0xD522, 0x02);
        LCD_WriteReg(0xD523, 0xA4);
        LCD_WriteReg(0xD524, 0x02);
        LCD_WriteReg(0xD525, 0xB7);
        LCD_WriteReg(0xD526, 0x02);
        LCD_WriteReg(0xD527, 0xCF);
        LCD_WriteReg(0xD528, 0x02);
        LCD_WriteReg(0xD529, 0xDE);
        LCD_WriteReg(0xD52A, 0x02);
        LCD_WriteReg(0xD52B, 0xF2);
        LCD_WriteReg(0xD52C, 0x02);
        LCD_WriteReg(0xD52D, 0xFE);
        LCD_WriteReg(0xD52E, 0x03);
        LCD_WriteReg(0xD52F, 0x10);
        LCD_WriteReg(0xD530, 0x03);
        LCD_WriteReg(0xD531, 0x33);
        LCD_WriteReg(0xD532, 0x03);
        LCD_WriteReg(0xD533, 0x6D);
        LCD_WriteReg(0xD600, 0x00);
        LCD_WriteReg(0xD601, 0x33);
        LCD_WriteReg(0xD602, 0x00);
        LCD_WriteReg(0xD603, 0x34);
        LCD_WriteReg(0xD604, 0x00);
        LCD_WriteReg(0xD605, 0x3A);
        LCD_WriteReg(0xD606, 0x00);
        LCD_WriteReg(0xD607, 0x4A);
        LCD_WriteReg(0xD608, 0x00);
        LCD_WriteReg(0xD609, 0x5C);
        LCD_WriteReg(0xD60A, 0x00);
        LCD_WriteReg(0xD60B, 0x81);

        LCD_WriteReg(0xD60C, 0x00);
        LCD_WriteReg(0xD60D, 0xA6);
        LCD_WriteReg(0xD60E, 0x00);
        LCD_WriteReg(0xD60F, 0xE5);
        LCD_WriteReg(0xD610, 0x01);
        LCD_WriteReg(0xD611, 0x13);
        LCD_WriteReg(0xD612, 0x01);
        LCD_WriteReg(0xD613, 0x54);
        LCD_WriteReg(0xD614, 0x01);
        LCD_WriteReg(0xD615, 0x82);
        LCD_WriteReg(0xD616, 0x01);
        LCD_WriteReg(0xD617, 0xCA);
        LCD_WriteReg(0xD618, 0x02);
        LCD_WriteReg(0xD619, 0x00);
        LCD_WriteReg(0xD61A, 0x02);
        LCD_WriteReg(0xD61B, 0x01);
        LCD_WriteReg(0xD61C, 0x02);
        LCD_WriteReg(0xD61D, 0x34);
        LCD_WriteReg(0xD61E, 0x02);
        LCD_WriteReg(0xD61F, 0x67);
        LCD_WriteReg(0xD620, 0x02);
        LCD_WriteReg(0xD621, 0x84);
        LCD_WriteReg(0xD622, 0x02);
        LCD_WriteReg(0xD623, 0xA4);
        LCD_WriteReg(0xD624, 0x02);
        LCD_WriteReg(0xD625, 0xB7);
        LCD_WriteReg(0xD626, 0x02);
        LCD_WriteReg(0xD627, 0xCF);
        LCD_WriteReg(0xD628, 0x02);
        LCD_WriteReg(0xD629, 0xDE);
        LCD_WriteReg(0xD62A, 0x02);
        LCD_WriteReg(0xD62B, 0xF2);
        LCD_WriteReg(0xD62C, 0x02);
        LCD_WriteReg(0xD62D, 0xFE);
        LCD_WriteReg(0xD62E, 0x03);
        LCD_WriteReg(0xD62F, 0x10);
        LCD_WriteReg(0xD630, 0x03);
        LCD_WriteReg(0xD631, 0x33);
        LCD_WriteReg(0xD632, 0x03);
        LCD_WriteReg(0xD633, 0x6D);
        
        LCD_WriteReg(0xF000, 0x55);
        LCD_WriteReg(0xF001, 0xAA);
        LCD_WriteReg(0xF002, 0x52);
        LCD_WriteReg(0xF003, 0x08);
        LCD_WriteReg(0xF004, 0x00);
        
        LCD_WriteReg(0xB100, 0xCC);
        LCD_WriteReg(0xB101, 0x00);
        
        LCD_WriteReg(0xB600, 0x05);
        
        LCD_WriteReg(0xB700, 0x70);
        LCD_WriteReg(0xB701, 0x70);
        
        LCD_WriteReg(0xB800, 0x01);
        LCD_WriteReg(0xB801, 0x03);
        LCD_WriteReg(0xB802, 0x03);
        LCD_WriteReg(0xB803, 0x03);
        
        LCD_WriteReg(0xBC00, 0x02);
        LCD_WriteReg(0xBC01, 0x00);
        LCD_WriteReg(0xBC02, 0x00);
        
        LCD_WriteReg(0xC900, 0xD0);
        LCD_WriteReg(0xC901, 0x02);
        LCD_WriteReg(0xC902, 0x50);
        LCD_WriteReg(0xC903, 0x50);
        LCD_WriteReg(0xC904, 0x50);
        LCD_WriteReg(0x3500, 0x00);
        LCD_WriteReg(0x3A00, 0x55); 
        LCD_WR_REG(0x1100);
        LCD_DelayUs(120);
        LCD_WR_REG(0x2900);
    }
    else if (lcddev.id == 0x7796)
    {
        LCD_WR_REG(0x11);

        HAL_Delay(120); 

        LCD_WR_REG(0x36); /* Memory Data Access Control MY,MX~~ */
        LCD_WR_DATA(0x48);
        
        LCD_WR_REG(0x3A);
        LCD_WR_DATA(0x55);
        
        LCD_WR_REG(0xF0);
        LCD_WR_DATA(0xC3);
        
        LCD_WR_REG(0xF0);
        LCD_WR_DATA(0x96);

        LCD_WR_REG(0xB4);
        LCD_WR_DATA(0x01);
        
        LCD_WR_REG(0xB6); /* Display Function Control */
        LCD_WR_DATA(0x0A);
        LCD_WR_DATA(0xA2);

        LCD_WR_REG(0xB7);
        LCD_WR_DATA(0xC6);

        LCD_WR_REG(0xB9);
        LCD_WR_DATA(0x02);
        LCD_WR_DATA(0xE0);

        LCD_WR_REG(0xC0);
        LCD_WR_DATA(0x80);
        LCD_WR_DATA(0x16);

        LCD_WR_REG(0xC1);
        LCD_WR_DATA(0x19);

        LCD_WR_REG(0xC2);
        LCD_WR_DATA(0xA7);

        LCD_WR_REG(0xC5);
        LCD_WR_DATA(0x16);   

        LCD_WR_REG(0xE8);
        LCD_WR_DATA(0x40);
        LCD_WR_DATA(0x8A);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x29);
        LCD_WR_DATA(0x19);
        LCD_WR_DATA(0xA5);
        LCD_WR_DATA(0x33);

        LCD_WR_REG(0xE0);
        LCD_WR_DATA(0xF0);
        LCD_WR_DATA(0x07);
        LCD_WR_DATA(0x0D);
        LCD_WR_DATA(0x04);
        LCD_WR_DATA(0x05);
        LCD_WR_DATA(0x14);
        LCD_WR_DATA(0x36);
        LCD_WR_DATA(0x54);
        LCD_WR_DATA(0x4C);
        LCD_WR_DATA(0x38);
        LCD_WR_DATA(0x13);
        LCD_WR_DATA(0x14);
        LCD_WR_DATA(0x2E);
        LCD_WR_DATA(0x34);

        LCD_WR_REG(0xE1);
        LCD_WR_DATA(0xF0);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x14);
        LCD_WR_DATA(0x0E);
        LCD_WR_DATA(0x0C);
        LCD_WR_DATA(0x08);
        LCD_WR_DATA(0x35);
        LCD_WR_DATA(0x44);
        LCD_WR_DATA(0x4C);
        LCD_WR_DATA(0x26);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x12);
        LCD_WR_DATA(0x2C);
        LCD_WR_DATA(0x32);

        LCD_WR_REG(0xF0);
        LCD_WR_DATA(0x3C);

        LCD_WR_REG(0xF0);
        LCD_WR_DATA(0x69);

        HAL_Delay(120);

        LCD_WR_REG(0x21);
        LCD_WR_REG(0x29);
    }
    else if (lcddev.id == 0x9806)
    {
        LCD_WR_REG(0xFF); /* EXTC Command Set enable register */
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0x98);
        LCD_WR_DATA(0x06);

        LCD_WR_REG(0xBC); /* GIP 1 */
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x0F);
        LCD_WR_DATA(0x61);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x0B);
        LCD_WR_DATA(0x10);
        LCD_WR_DATA(0x37);
        LCD_WR_DATA(0x63);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0x52);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x40);

        LCD_WR_REG(0xBD); /* GIP 2 */
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x23);
        LCD_WR_DATA(0x45);
        LCD_WR_DATA(0x67);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0x23);
        LCD_WR_DATA(0x45);
        LCD_WR_DATA(0x67);

        LCD_WR_REG(0xBE); /* GIP 3 */
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0xAB);
        LCD_WR_DATA(0x60);
        LCD_WR_DATA(0x22);
        LCD_WR_DATA(0x22);
        LCD_WR_DATA(0x22);
        LCD_WR_DATA(0x22);
        LCD_WR_DATA(0x22);

        LCD_WR_REG(0xC7); /* VCOM Control */
        LCD_WR_DATA(0x36);

        LCD_WR_REG(0xED); /* EN_volt_reg VGMP / VGMN /VGSP / VGSN voltage to output */
        LCD_WR_DATA(0x7F);
        LCD_WR_DATA(0x0F);

        LCD_WR_REG(0XC0); /* Power Control 1 Setting AVDD / AVEE / VGH / VGL */
        LCD_WR_DATA(0x0F);
        LCD_WR_DATA(0x0B);
        LCD_WR_DATA(0x0A);  /* VGH 15V,VGLO-10V */

        LCD_WR_REG(0XFC); /* AVDD / AVEE generated by internal pumping. */
        LCD_WR_DATA(0x08);

        LCD_WR_REG(0XDF); 
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x20);

        LCD_WR_REG(0XF3); /* DVDD Voltage Setting */
        LCD_WR_DATA(0x74);

        LCD_WR_REG(0xB4); /* Inversion Type */
        LCD_WR_DATA(0x00);  /* 02 */
        LCD_WR_DATA(0x00);  /* 02 */
        LCD_WR_DATA(0x00);  /* 02 */

        LCD_WR_REG(0xF7); /* Resolution Control */
        LCD_WR_DATA(0x82);  /* 480*800 */

        LCD_WR_REG(0xB1); /* FRAME RATE Setting */
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x13);
        LCD_WR_DATA(0x13); 

        LCD_WR_REG(0XF2); /* CR_EQ_PC_SDT  #C0,06,40,28 */
        LCD_WR_DATA(0x80);
        LCD_WR_DATA(0x04);
        LCD_WR_DATA(0x40);
        LCD_WR_DATA(0x28);

        LCD_WR_REG(0XC1); /* Power Control 2  SD OP Bias_VRH1_VRH2_EXT_CPCK_SEL */
        LCD_WR_DATA(0x17);
        LCD_WR_DATA(0x88);  /* VGMP */
        LCD_WR_DATA(0x88);  /* VGMN */
        LCD_WR_DATA(0x20);

        LCD_WR_REG(0xE0); /* Positive Gamma Control */
        LCD_WR_DATA(0x00);  /* P1 */
        LCD_WR_DATA(0x0A);  /* P2 */
        LCD_WR_DATA(0x12);  /* P3 */
        LCD_WR_DATA(0x10);  /* P4 */
        LCD_WR_DATA(0x0E);  /* P5 */
        LCD_WR_DATA(0x20);  /* P6 */
        LCD_WR_DATA(0xCC);  /* P7 */
        LCD_WR_DATA(0x07);  /* P8 */
        LCD_WR_DATA(0x06);  /* P9 */
        LCD_WR_DATA(0x0B);  /* P10 */
        LCD_WR_DATA(0x0E);  /* P11 */
        LCD_WR_DATA(0x0F);  /* P12 */
        LCD_WR_DATA(0x0D);  /* P13 */
        LCD_WR_DATA(0x15);  /* P14 */
        LCD_WR_DATA(0x10);  /* P15 */
        LCD_WR_DATA(0x00);  /* P16 */

        LCD_WR_REG(0xE1); /* Negative Gamma Correction */
        LCD_WR_DATA(0x00);  /* P1 */
        LCD_WR_DATA(0x0B);  /* P2 */
        LCD_WR_DATA(0x13);  /* P3 */
        LCD_WR_DATA(0x0D);  /* P4 */
        LCD_WR_DATA(0x0E);  /* P5 */
        LCD_WR_DATA(0x1B);  /* P6 */
        LCD_WR_DATA(0x71);  /* P7 */
        LCD_WR_DATA(0x06);  /* P8 */
        LCD_WR_DATA(0x06);  /* P9 */
        LCD_WR_DATA(0x0A);  /* P10 */
        LCD_WR_DATA(0x0F);  /* P11 */
        LCD_WR_DATA(0x0E);  /* P12 */
        LCD_WR_DATA(0x0F);  /* P13 */
        LCD_WR_DATA(0x15);  /* P14 */
        LCD_WR_DATA(0x0C);  /* P15 */
        LCD_WR_DATA(0x00);  /* P16 */

        LCD_WR_REG(0x2a);   
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x01);
        LCD_WR_DATA(0xdf);

        LCD_WR_REG(0x2b);   
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x03);
        LCD_WR_DATA(0x1f);

        LCD_WR_REG(0x3A); /* Pixel Format */
        LCD_WR_DATA(0x55);

        LCD_WR_REG(0x36); /* Memory Access Control */
        LCD_WR_DATA(0x00);  /* 02-180 */

        LCD_WR_REG(0x11);
        HAL_Delay(120);   
        LCD_WR_REG(0x29);  
        HAL_Delay(20);  
        LCD_WR_REG(0x2C);
    }
    else if (lcddev.id == 0X1963)
    {
        LCD_WR_REG(0xE2);		
        LCD_WR_DATA(0x1D);		
        LCD_WR_DATA(0x02);		
        LCD_WR_DATA(0x04);		
        LCD_DelayUs(100);
        LCD_WR_REG(0xE0);		
        LCD_WR_DATA(0x01);		
        HAL_Delay(10);
        LCD_WR_REG(0xE0);		
        LCD_WR_DATA(0x03);		
        HAL_Delay(12);
        LCD_WR_REG(0x01);		
        HAL_Delay(10);

        LCD_WR_REG(0xE6);		
        LCD_WR_DATA(0x2F);
        LCD_WR_DATA(0xFF);
        LCD_WR_DATA(0xFF);

        LCD_WR_REG(0xB0);		
        LCD_WR_DATA(0x20);		
        LCD_WR_DATA(0x00);		

        LCD_WR_DATA((SSD_HOR_RESOLUTION - 1) >> 8); 
        LCD_WR_DATA(SSD_HOR_RESOLUTION - 1);
        LCD_WR_DATA((SSD_VER_RESOLUTION - 1) >> 8); 
        LCD_WR_DATA(SSD_VER_RESOLUTION - 1);
        LCD_WR_DATA(0x00);		

        LCD_WR_REG(0xB4);		
        LCD_WR_DATA((SSD_HT - 1) >> 8);
        LCD_WR_DATA(SSD_HT - 1);
        LCD_WR_DATA(SSD_HPS >> 8);
        LCD_WR_DATA(SSD_HPS);
        LCD_WR_DATA(SSD_HOR_PULSE_WIDTH - 1);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);
        LCD_WR_REG(0xB6);		
        LCD_WR_DATA((SSD_VT - 1) >> 8);
        LCD_WR_DATA(SSD_VT - 1);
        LCD_WR_DATA(SSD_VPS >> 8);
        LCD_WR_DATA(SSD_VPS);
        LCD_WR_DATA(SSD_VER_FRONT_PORCH - 1);
        LCD_WR_DATA(0x00);
        LCD_WR_DATA(0x00);

        LCD_WR_REG(0xF0);	
        LCD_WR_DATA(0x03);	

        LCD_WR_REG(0x29);	
        
        LCD_WR_REG(0xD0);	
        LCD_WR_DATA(0x00);	

        LCD_WR_REG(0xBE);	
        LCD_WR_DATA(0x05);	
        LCD_WR_DATA(0xFE);	
        LCD_WR_DATA(0x01);	
        LCD_WR_DATA(0x00);	
        LCD_WR_DATA(0x00);	
        LCD_WR_DATA(0x00);	

        LCD_WR_REG(0xB8);	
        LCD_WR_DATA(0x03);	
        LCD_WR_DATA(0x01);	
        LCD_WR_REG(0xBA);
        LCD_WR_DATA(0X01);	

        LCD_SSD_BackLightSet(100);
    }


    LCD_Display_Dir(1);
    HAL_GPIO_WritePin(LCD_LED_GPIO_Port, LCD_LED_Pin, GPIO_PIN_SET);
    LCD_Clear(WHITE);
}


void LCD_Clear(u32 color)
{
    /* 设置全屏窗口后连续写入 width*height 个像素，效率高于逐点定位。 */
    u32 index = 0;
    /* 一次设置起始地址后连续写显存，是全屏填充最快且通用的方式。 */
    u32 totalpoint = lcddev.width;
    totalpoint *= lcddev.height; 			
    LCD_SetCursor(0x00, 0x0000);			
    LCD_WriteRAM_Prepare();     		

    for (index = 0; index < totalpoint; index++)
    {
        LCD->LCD_RAM = color;
    }
}



void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u32 color)
{
    /* 填充闭区间矩形 [sx,ex] × [sy,ey]。 */
    u32 pixels;

    if (ex < sx || ey < sy || sx >= lcddev.width || sy >= lcddev.height)
    {
        return;
    }
    if (ex >= lcddev.width) ex = lcddev.width - 1U;
    if (ey >= lcddev.height) ey = lcddev.height - 1U;

    LCD_Set_Window(sx, sy, ex - sx + 1U, ey - sy + 1U);
    LCD_WriteRAM_Prepare();
    pixels = (u32)(ex - sx + 1U) * (ey - sy + 1U);
    while (pixels-- != 0U) LCD->LCD_RAM = color;
}



void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color)
{
    /* 使用外部 RGB565 数组填充矩形，数组按行优先排列。 */
    u16 height, width;
    u16 i, j;
    width = ex - sx + 1; 			
    height = ey - sy + 1;			

    for (i = 0; i < height; i++)
    {
        LCD_SetCursor(sx, sy + i);   	
        LCD_WriteRAM_Prepare();     

        for (j = 0; j < width; j++)LCD->LCD_RAM = color[i * width + j]; 
    }
}



void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2)
{
    /* 使用整数误差累积算法绘制任意方向直线。 */
    u16 t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;

    if (y1 == y2)
    {
        if (x2 < x1)
        {
            t = x1;
            x1 = x2;
            x2 = t;
        }
        LCD_Fill(x1, y1, x2, y1, POINT_COLOR);
        return;
    }
    if (x1 == x2)
    {
        if (y2 < y1)
        {
            t = y1;
            y1 = y2;
            y2 = t;
        }
        LCD_Fill(x1, y1, x1, y2, POINT_COLOR);
        return;
    }
    delta_x = x2 - x1; 
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;

    if (delta_x > 0)incx = 1; 
    else if (delta_x == 0)incx = 0; 
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }

    if (delta_y > 0)incy = 1;
    else if (delta_y == 0)incy = 0; 
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }

    if ( delta_x > delta_y)distance = delta_x; 
    else distance = delta_y;

    for (t = 0; t <= distance + 1; t++ ) 
    {
        LCD_DrawPoint(uRow, uCol); 
        xerr += delta_x ;
        yerr += delta_y ;

        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }

        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}


void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2)
{
    /* 由四条直线组成空心矩形。 */
    LCD_DrawLine(x1, y1, x2, y1);
    LCD_DrawLine(x1, y1, x1, y2);
    LCD_DrawLine(x1, y2, x2, y2);
    LCD_DrawLine(x2, y1, x2, y2);
}



void LCD_Draw_Circle(u16 x0, u16 y0, u8 r)
{
    /* 使用八对称 Bresenham 算法绘制空心圆。 */
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);       

    while (a <= b)
    {
        LCD_DrawPoint(x0 + a, y0 - b);        
        LCD_DrawPoint(x0 + b, y0 - a);        
        LCD_DrawPoint(x0 + b, y0 + a);        
        LCD_DrawPoint(x0 + a, y0 + b);        
        LCD_DrawPoint(x0 - a, y0 + b);        
        LCD_DrawPoint(x0 - b, y0 + a);
        LCD_DrawPoint(x0 - a, y0 - b);        
        LCD_DrawPoint(x0 - b, y0 - a);        
        a++;

        
        if (di < 0)di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }
    }
}

uint16_t LCD_GetID(void)
{
    /* 返回 LCD_Init 阶段识别出的控制器 ID。 */
    return lcddev.id;
}

uint8_t LCD_IsSupported(void)
{
    /* 判断当前控制器是否存在已适配的初始化和绘图分支。 */
    return lcddev.id == 0X9341U || lcddev.id == 0X5310U ||
           lcddev.id == 0X5510U || lcddev.id == 0X1963U ||
           lcddev.id == 0X7789U || lcddev.id == 0X7796U ||
           lcddev.id == 0X9806U;
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    /* 以两个角点形式设置窗口，便于应用层调用。 */
    if (x1 < x0 || y1 < y0)
    {
        return;
    }
    LCD_Set_Window(x0, y0, x1 - x0 + 1U, y1 - y0 + 1U);
}

void LCD_TestPattern(void)
{
    /* 四色块硬件自检：可快速判断 LCD 地址、总线宽度和背光是否正常。 */
    uint16_t halfWidth;
    uint16_t halfHeight;

    if (LCD_IsSupported() == 0U)
    {
        return;
    }

    halfWidth = lcddev.width / 2U;
    halfHeight = lcddev.height / 2U;
    LCD_Fill(0U, 0U, halfWidth - 1U, halfHeight - 1U, RED);
    LCD_Fill(halfWidth, 0U, lcddev.width - 1U, halfHeight - 1U, GREEN);
    LCD_Fill(0U, halfHeight, halfWidth - 1U, lcddev.height - 1U, BLUE);
    LCD_Fill(halfWidth, halfHeight, lcddev.width - 1U, lcddev.height - 1U, WHITE);
    POINT_COLOR = BLACK;
    LCD_DrawRectangle(0U, 0U, lcddev.width - 1U, lcddev.height - 1U);
}
