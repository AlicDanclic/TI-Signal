#ifndef LCD_H
#define LCD_H

#include "main.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef volatile uint16_t vu16;

typedef struct
{
    u16 width;
    u16 height;
    u16 id;
    u8 dir;
    u16 wramcmd;
    u16 setxcmd;
    u16 setycmd;
} _lcd_dev;

typedef struct
{
    vu16 LCD_REG;
    vu16 LCD_RAM;
} LCD_TypeDef;

#define LCD_WIDTH 800U
#define LCD_HEIGHT 480U
/* Bank1 的 NE1 基地址为 0x60000000；A18 接 RS，16 位总线需左移一位。 */
#define LCD_BASE (0x60000000UL | 0x0007FFFEUL)
#define LCD ((LCD_TypeDef *)LCD_BASE)

#define L2R_U2D 0U
#define L2R_D2U 1U
#define R2L_U2D 2U
#define R2L_D2U 3U
#define U2D_L2R 4U
#define U2D_R2L 5U
#define D2U_L2R 6U
#define D2U_R2L 7U
#define DFT_SCAN_DIR L2R_U2D

#define WHITE 0xFFFFU
#define BLACK 0x0000U
#define BLUE 0x001FU
#define BRED 0xF81FU
#define GRED 0xFFE0U
#define GBLUE 0x07FFU
#define RED 0xF800U
#define MAGENTA 0xF81FU
#define GREEN 0x07E0U
#define CYAN 0x7FFFU
#define YELLOW 0xFFE0U
#define BROWN 0xBC40U
#define BRRED 0xFC07U
#define GRAY 0x8430U

#define SSD_HOR_RESOLUTION 800U
#define SSD_VER_RESOLUTION 480U
#define SSD_HOR_PULSE_WIDTH 1U
#define SSD_HOR_BACK_PORCH 46U
#define SSD_HOR_FRONT_PORCH 210U
#define SSD_VER_PULSE_WIDTH 1U
#define SSD_VER_BACK_PORCH 23U
#define SSD_VER_FRONT_PORCH 22U
#define SSD_HT (SSD_HOR_RESOLUTION + SSD_HOR_BACK_PORCH + SSD_HOR_FRONT_PORCH)
#define SSD_HPS SSD_HOR_BACK_PORCH
#define SSD_VT (SSD_VER_RESOLUTION + SSD_VER_BACK_PORCH + SSD_VER_FRONT_PORCH)
#define SSD_VPS SSD_VER_BACK_PORCH

extern _lcd_dev lcddev;
extern u32 POINT_COLOR;
extern u32 BACK_COLOR;

/* 底层总线访问函数：通常仅在 LCD 驱动内部或调试控制器寄存器时使用。 */
void LCD_WR_REG(vu16 regval);
void LCD_WR_DATA(vu16 data);
u16 LCD_RD_DATA(void);
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u16 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);
/* 初始化时自动识别控制器并执行对应寄存器序列。 */
void LCD_Init(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);
/* 基础绘图函数。坐标原点为横屏左上角，颜色格式为 RGB565。 */
void LCD_Clear(u32 color);
void LCD_SetCursor(u16 Xpos, u16 Ypos);
void LCD_DrawPoint(u16 x, u16 y);
void LCD_Fast_DrawPoint(u16 x, u16 y, u32 color);
u32 LCD_ReadPoint(u16 x, u16 y);
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u32 color);
void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color);
void LCD_SSD_BackLightSet(u8 pwm);
void LCD_Scan_Dir(u8 dir);
void LCD_Display_Dir(u8 dir);
void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height);

/* 查询初始化阶段识别出的控制器 ID 与兼容状态。 */
uint16_t LCD_GetID(void);
uint8_t LCD_IsSupported(void);
void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
/* 四色块硬件自检图，排查 FMC 连线时使用。 */
void LCD_TestPattern(void);

#endif
