#include "waveform.h"

#include "lcd.h"

/*
 * 示波器界面采用局部刷新：网格只在初始化和擦除曲线后恢复，
 * 每帧只处理旧曲线、新曲线、测量数值和游标叠加层，以降低 FMC 写入量。
 */

#define PLOT_LEFT       52U
#define PLOT_TOP        58U
#define PLOT_WIDTH      540U
#define PLOT_HEIGHT     360U
#define PANEL_LEFT      610U
#define SAMPLE_COUNT    WAVEFORM_SAMPLE_COUNT
#define ADC_CLOCK_HZ    50000000UL
#define ADC_CYCLES_US   50UL
#define LINK_TIMEOUT_MS 2000UL
#define MEASUREMENT_REFRESH_MS 100UL
#define TIME_ZOOM_LEVELS 4U

#define COLOR_SCREEN_BG 0x0861U
#define COLOR_PLOT_BG   0x0021U
#define COLOR_GRID      0x18E3U
#define COLOR_GRID_MAIN 0x31A6U
#define COLOR_PANEL     0x10A2U
#define COLOR_TEXT      0xD6BAU
#define COLOR_MUTED     0x7BEFU
#define COLOR_ACCENT    0x07FFU
#define COLOR_WAVE      0xFFE0U
#define COLOR_CURSOR    0xF81FU

static uint16_t samples[SAMPLE_COUNT];
static uint16_t previousSamples[SAMPLE_COUNT];
static uint16_t pendingSamples[SAMPLE_COUNT];
static uint32_t frequencyHz;
static uint32_t sampleDiv;
static uint32_t sampleRateHz;
static uint16_t peakToPeakMv;
static int32_t offsetMv;
static uint16_t rmsMv;
static uint32_t pendingFrequencyHz;
static uint32_t pendingSampleDiv;
static uint32_t pendingPeriodCycles;
static uint8_t touchEnabled;
static uint8_t cursorEnabled;
static uint8_t cursorDrawn;
static uint16_t cursorX;
static uint16_t cursorY;
static uint8_t traceDrawn;
static uint8_t running = 1U;
static uint8_t framePending;
static uint8_t metadataPending;
static uint8_t frameAvailable;
static uint8_t signalFlags;
static uint8_t frameSequence;
static uint8_t pendingFlags;
static uint8_t pendingSequence;
static uint8_t linkTimedOut;
static uint32_t lastFrameTickMs;
static uint16_t lastTouchX;
static uint16_t lastTouchY;
static uint8_t timeZoomIndex;
static uint32_t linkBytesReceived;
static uint32_t linkValidFrames;
static uint32_t linkTotalErrors;
static uint32_t lastMeasurementDrawTickMs;
static uint8_t measurementDirty;
static uint32_t displayedSampleDiv;

/* 每个缩放级别显示的最后一个源采样点：约 2.8、2、1、0.5 个周期。 */
static const uint8_t timebaseLastSample[TIME_ZOOM_LEVELS] = {180U, 128U, 64U, 32U};

/* 5x7 点阵字库，仅包含本界面使用的数字、字母和符号。 */
static const char glyphCharacters[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ:.-+/()%";
static const uint8_t glyphRows[][7] = {
    {0,0,0,0,0,0,0},
    {14,17,19,21,25,17,14},{4,12,4,4,4,4,14},{14,17,1,2,4,8,31},
    {30,1,1,14,1,1,30},{2,6,10,18,31,2,2},{31,16,16,30,1,1,30},
    {14,16,16,30,17,17,14},{31,1,2,4,8,8,8},{14,17,17,14,17,17,14},
    {14,17,17,15,1,1,14},
    {14,17,17,31,17,17,17},{30,17,17,30,17,17,30},{14,17,16,16,16,17,14},
    {30,17,17,17,17,17,30},{31,16,16,30,16,16,31},{31,16,16,30,16,16,16},
    {14,17,16,23,17,17,15},{17,17,17,31,17,17,17},{14,4,4,4,4,4,14},
    {7,2,2,2,2,18,12},{17,18,20,24,20,18,17},{16,16,16,16,16,16,31},
    {17,27,21,21,17,17,17},{17,25,21,19,17,17,17},{14,17,17,17,17,17,14},
    {30,17,17,30,16,16,16},{14,17,17,17,21,18,13},{30,17,17,30,20,18,17},
    {15,16,16,14,1,1,30},{31,4,4,4,4,4,4},{17,17,17,17,17,17,14},
    {17,17,17,17,17,10,4},{17,17,17,21,21,21,10},{17,17,10,4,10,17,17},
    {17,17,10,4,4,4,4},{31,1,2,4,8,16,31},
    {0,4,0,0,4,0,0},{0,0,0,0,0,4,4},{0,0,0,31,0,0,0},
    {0,4,4,31,4,4,0},{1,2,4,8,16,0,0},{14,17,2,4,8,17,14},
    {6,9,10,4,10,9,6},{24,25,2,4,8,19,3}
};

static const uint8_t *Waveform_FindGlyph(char character)
{
    /* 从精简 5x7 ASCII 字库中查找界面文字对应的点阵。 */
    uint16_t index;
    for (index = 0U; glyphCharacters[index] != '\0'; index++)
    {
        if (glyphCharacters[index] == character)
        {
            return glyphRows[index];
        }
    }
    return glyphRows[0];
}

static void Waveform_DrawCharacter(uint16_t x, uint16_t y, char character,
                                   uint16_t color, uint8_t scale)
{
    /* 按比例绘制一个 ASCII 字符，避免引入完整中文字体占用 Flash。 */
    const uint8_t *rows = Waveform_FindGlyph(character);
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < 7U; row++)
    {
        for (column = 0U; column < 5U; column++)
        {
            if ((rows[row] & (1U << (4U - column))) != 0U)
            {
                LCD_Fill(x + column * scale, y + row * scale,
                         x + column * scale + scale - 1U,
                         y + row * scale + scale - 1U, color);
            }
        }
    }
}

static void Waveform_DrawText(uint16_t x, uint16_t y, const char *text,
                              uint16_t color, uint8_t scale)
{
    /* 连续绘制字符串，字符间距为 1 个缩放单位。 */
    while (*text != '\0')
    {
        Waveform_DrawCharacter(x, y, *text, color, scale);
        x += 6U * scale;
        text++;
    }
}

static void Waveform_UintToText(uint32_t value, char *text, uint8_t digits)
{
    /* 将无符号数转换为固定宽度十进制字符串，适合仪表盘显示。 */
    uint8_t index;
    for (index = 0U; index < digits; index++)
    {
        text[digits - index - 1U] = (char)('0' + value % 10U);
        value /= 10U;
    }
    text[digits] = '\0';
}

static uint8_t Waveform_UintToCompactText(uint32_t value, char *text)
{
    uint8_t digits = 0U;
    uint8_t index;
    char temporary;

    do
    {
        text[digits] = (char)('0' + value % 10U);
        digits++;
        value /= 10U;
    } while (value != 0U);

    for (index = 0U; index < digits / 2U; index++)
    {
        temporary = text[index];
        text[index] = text[digits - index - 1U];
        text[digits - index - 1U] = temporary;
    }
    text[digits] = '\0';
    return digits;
}

static void Waveform_DrawValue(uint16_t x, uint16_t y, uint32_t value,
                               uint8_t digits, const char *unit, uint16_t color)
{
    char text[11];
    Waveform_UintToText(value, text, digits);
    Waveform_DrawText(x, y, text, color, 2U);
    Waveform_DrawText((uint16_t)(x + digits * 12U + 5U), y + 7U, unit, COLOR_MUTED, 1U);
}

static void Waveform_DrawUnsigned(uint16_t x, uint16_t y, uint32_t value,
                                  const char *unit, uint16_t color, uint8_t scale)
{
    char text[11];
    uint8_t digits = Waveform_UintToCompactText(value, text);

    Waveform_DrawText(x, y, text, color, scale);
    Waveform_DrawText((uint16_t)(x + digits * 6U * scale + 5U),
                      (uint16_t)(y + (scale == 2U ? 7U : 0U)),
                      unit, COLOR_MUTED, 1U);
}

static void Waveform_DrawStaticLayout(void)
{
    /* 绘制不会随采样变化的标题栏、测量面板和控制按钮。 */
    LCD_Clear(COLOR_SCREEN_BG);
    LCD_Fill(0U, 0U, 799U, 39U, 0x014BU);
    LCD_Fill(0U, 39U, 799U, 41U, COLOR_ACCENT);
    Waveform_DrawText(18U, 11U, "FPGA SIGNAL SCOPE", WHITE, 2U);
    LCD_Fill(618U, 5U, 786U, 35U, 0x0861U);
    POINT_COLOR = COLOR_ACCENT;
    LCD_DrawRectangle(618U, 5U, 786U, 35U);
    Waveform_DrawText(638U, 14U, "PAUSE", WHITE, 1U);

    LCD_Fill(PANEL_LEFT, 52U, 789U, 466U, COLOR_PANEL);
    POINT_COLOR = COLOR_GRID_MAIN;
    LCD_DrawRectangle(PANEL_LEFT, 52U, 789U, 466U);
    Waveform_DrawText(628U, 66U, "MEASURE", COLOR_ACCENT, 2U);
    Waveform_DrawText(626U, 108U, "FREQ", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 158U, "VPP", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 208U, "RMS", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 258U, "OFFSET", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 318U, "SAMPLE", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 370U, "TOUCH", COLOR_MUTED, 1U);
    Waveform_DrawText(626U, 394U, touchEnabled != 0U ? "READY WAIT" : "NOT FOUND",
                      touchEnabled != 0U ? GREEN : YELLOW, 1U);
    Waveform_DrawText(626U, 430U, "WAIT FPGA", YELLOW, 1U);
}

static void Waveform_DrawGrid(void)
{
    /* 首次绘制示波器背景，包括次网格、主网格、边框和坐标刻度。 */
    uint16_t x;
    uint16_t y;
    uint16_t right = PLOT_LEFT + PLOT_WIDTH;
    uint16_t bottom = PLOT_TOP + PLOT_HEIGHT;

    LCD_Fill(PLOT_LEFT, PLOT_TOP, right, bottom, COLOR_PLOT_BG);
    POINT_COLOR = COLOR_GRID;
    for (x = PLOT_LEFT + 27U; x < right; x += 27U)
    {
        LCD_DrawLine(x, PLOT_TOP, x, bottom);
    }
    for (y = PLOT_TOP + 18U; y < bottom; y += 18U)
    {
        LCD_DrawLine(PLOT_LEFT, y, right, y);
    }
    POINT_COLOR = COLOR_GRID_MAIN;
    for (x = PLOT_LEFT; x <= right; x += 54U)
    {
        LCD_DrawLine(x, PLOT_TOP, x, bottom);
    }
    for (y = PLOT_TOP; y <= bottom; y += 36U)
    {
        LCD_DrawLine(PLOT_LEFT, y, right, y);
    }
    POINT_COLOR = WHITE;
    LCD_DrawRectangle(PLOT_LEFT, PLOT_TOP, right, bottom);
    LCD_DrawLine(PLOT_LEFT, PLOT_TOP + PLOT_HEIGHT / 2U, right,
                 PLOT_TOP + PLOT_HEIGHT / 2U);

    Waveform_DrawText(4U, PLOT_TOP - 3U, "+5V", COLOR_MUTED, 1U);
    Waveform_DrawText(16U, PLOT_TOP + PLOT_HEIGHT / 2U - 3U, "0V", COLOR_MUTED, 1U);
    Waveform_DrawText(4U, PLOT_TOP + PLOT_HEIGHT - 6U, "-5V", COLOR_MUTED, 1U);
}

static void Waveform_RedrawGridLines(void)
{
    /* 擦除曲线后恢复被覆盖的网格线，不重新填充整个绘图区。 */
    uint16_t x;
    uint16_t y;
    uint16_t right = PLOT_LEFT + PLOT_WIDTH;
    uint16_t bottom = PLOT_TOP + PLOT_HEIGHT;

    POINT_COLOR = COLOR_GRID;
    for (x = PLOT_LEFT + 27U; x < right; x += 27U)
    {
        LCD_DrawLine(x, PLOT_TOP, x, bottom);
    }
    for (y = PLOT_TOP + 18U; y < bottom; y += 18U)
    {
        LCD_DrawLine(PLOT_LEFT, y, right, y);
    }
    POINT_COLOR = COLOR_GRID_MAIN;
    for (x = PLOT_LEFT; x <= right; x += 54U)
    {
        LCD_DrawLine(x, PLOT_TOP, x, bottom);
    }
    for (y = PLOT_TOP; y <= bottom; y += 36U)
    {
        LCD_DrawLine(PLOT_LEFT, y, right, y);
    }
    POINT_COLOR = WHITE;
    LCD_DrawRectangle(PLOT_LEFT, PLOT_TOP, right, bottom);
    LCD_DrawLine(PLOT_LEFT, PLOT_TOP + PLOT_HEIGHT / 2U, right,
                 PLOT_TOP + PLOT_HEIGHT / 2U);
}

static uint32_t Waveform_Isqrt64(uint64_t value)
{
    uint64_t result = 0U;
    uint64_t bit = (uint64_t)1U << 62U;

    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1U) + bit;
        }
        else
        {
            result >>= 1U;
        }
        bit >>= 2U;
    }
    return (uint32_t)result;
}

static int32_t Waveform_CodeToMillivolts(uint16_t code)
{
    return (int32_t)(((uint32_t)code * 10000U + 2047U) / 4095U) - 5000;
}

static void Waveform_ComputeMeasurements(void)
{
    uint16_t index;
    uint16_t minimum = 4095U;
    uint16_t maximum = 0U;
    int64_t sum = 0;
    uint64_t sumSquares = 0U;
    int32_t millivolts;

    for (index = 0U; index < SAMPLE_COUNT; index++)
    {
        if (samples[index] < minimum)
        {
            minimum = samples[index];
        }
        if (samples[index] > maximum)
        {
            maximum = samples[index];
        }
        millivolts = Waveform_CodeToMillivolts(samples[index]);
        sum += millivolts;
    }

    peakToPeakMv = (uint16_t)(((uint32_t)(maximum - minimum) * 10000U + 2047U) /
                              4095U);
    offsetMv = (int32_t)(sum / (int32_t)SAMPLE_COUNT);
    for (index = 0U; index < SAMPLE_COUNT; index++)
    {
        millivolts = Waveform_CodeToMillivolts(samples[index]) - offsetMv;
        sumSquares += (uint64_t)((int64_t)millivolts * millivolts);
    }
    rmsMv = (uint16_t)Waveform_Isqrt64(sumSquares / SAMPLE_COUNT);
}

static void Waveform_DrawTimebase(void)
{
    uint64_t spanUs64;
    uint32_t spanUs;
    uint8_t lastSample = timebaseLastSample[timeZoomIndex];

    displayedSampleDiv = sampleDiv;

    LCD_Fill(PLOT_LEFT, 425U, PLOT_LEFT + PLOT_WIDTH, 476U, COLOR_SCREEN_BG);
    LCD_Fill(PLOT_LEFT, 430U, PLOT_LEFT + 78U, 472U, COLOR_PANEL);
    LCD_Fill(PLOT_LEFT + PLOT_WIDTH - 78U, 430U,
             PLOT_LEFT + PLOT_WIDTH, 472U, COLOR_PANEL);
    POINT_COLOR = timeZoomIndex > 0U ? COLOR_ACCENT : COLOR_MUTED;
    LCD_DrawRectangle(PLOT_LEFT, 430U, PLOT_LEFT + 78U, 472U);
    POINT_COLOR = timeZoomIndex + 1U < TIME_ZOOM_LEVELS ?
                  COLOR_ACCENT : COLOR_MUTED;
    LCD_DrawRectangle(PLOT_LEFT + PLOT_WIDTH - 78U, 430U,
                      PLOT_LEFT + PLOT_WIDTH, 472U);
    Waveform_DrawText(PLOT_LEFT + 12U, 448U, "ZOOM-",
                      timeZoomIndex > 0U ? WHITE : COLOR_MUTED, 1U);
    Waveform_DrawText(PLOT_LEFT + PLOT_WIDTH - 66U, 448U, "ZOOM+",
                      timeZoomIndex + 1U < TIME_ZOOM_LEVELS ?
                      WHITE : COLOR_MUTED, 1U);
    if (frameAvailable == 0U)
    {
        Waveform_DrawText(260U, 448U, "WAIT FPGA", YELLOW, 1U);
        return;
    }
    if ((signalFlags & 0x01U) == 0U || sampleDiv == 0U)
    {
        Waveform_DrawText(262U, 448U, "NO SIGNAL", YELLOW, 1U);
        return;
    }

    spanUs64 = ((uint64_t)lastSample * sampleDiv +
                ADC_CYCLES_US / 2U) / ADC_CYCLES_US;
    spanUs = spanUs64 > 0xFFFFFFFFULL ? 0xFFFFFFFFUL : (uint32_t)spanUs64;
    Waveform_DrawText(210U, 438U, "SPAN", COLOR_MUTED, 1U);
    Waveform_DrawUnsigned(250U, 438U, spanUs, "US", WHITE, 1U);
    Waveform_DrawText(222U, 458U, "ADC 50MHZ", COLOR_TEXT, 1U);
}

static void Waveform_DrawLinkState(void)
{
    char sequenceText[4];

    LCD_Fill(622U, 424U, 782U, 452U, COLOR_PANEL);
    if (frameAvailable == 0U)
    {
        Waveform_DrawText(626U, 430U, "WAIT FPGA", YELLOW, 1U);
        return;
    }

    if (linkTimedOut != 0U)
    {
        Waveform_DrawText(626U, 430U, "LINK LOST", RED, 1U);
        return;
    }

    Waveform_DrawText(626U, 430U,
                      (signalFlags & 0x01U) != 0U ? "FPGA LINK" : "NO SIGNAL",
                      (signalFlags & 0x01U) != 0U ? GREEN : YELLOW, 1U);
    Waveform_UintToText(frameSequence, sequenceText, 3U);
    Waveform_DrawText(720U, 430U, "S", COLOR_MUTED, 1U);
    Waveform_DrawText(732U, 430U, sequenceText, WHITE, 1U);
}

static void Waveform_DrawLinkStats(void)
{
    char text[4];

    LCD_Fill(622U, 452U, 782U, 465U, COLOR_PANEL);
    Waveform_DrawText(626U, 455U, "B", COLOR_MUTED, 1U);
    Waveform_UintToText(linkBytesReceived % 1000U, text, 3U);
    Waveform_DrawText(638U, 455U, text, WHITE, 1U);
    Waveform_DrawText(668U, 455U, "V", COLOR_MUTED, 1U);
    Waveform_UintToText(linkValidFrames % 1000U, text, 3U);
    Waveform_DrawText(680U, 455U, text, GREEN, 1U);
    Waveform_DrawText(710U, 455U, "E", COLOR_MUTED, 1U);
    Waveform_UintToText(linkTotalErrors % 1000U, text, 3U);
    Waveform_DrawText(722U, 455U, text,
                      linkTotalErrors == 0U ? WHITE : RED, 1U);
}

static uint16_t Waveform_SampleToY(uint16_t sample)
{
    return PLOT_TOP + PLOT_HEIGHT -
           (uint16_t)((uint32_t)sample * PLOT_HEIGHT / 4095U);
}

static void Waveform_DrawTraceData(const uint16_t *trace, uint16_t color)
{
    /* 将 12 位采样值映射到绘图区，并用折线连接相邻采样点。 */
    uint16_t index;
    uint16_t lastSample = timebaseLastSample[timeZoomIndex];
    uint16_t startSample = (SAMPLE_COUNT - 1U) - lastSample;
    uint16_t previousX = PLOT_LEFT;
    uint16_t previousY = Waveform_SampleToY(trace[startSample]);
    uint16_t x;
    uint16_t y;

    POINT_COLOR = color;
    for (index = 1U; index <= lastSample; index++)
    {
        x = PLOT_LEFT + (uint16_t)((uint32_t)index * PLOT_WIDTH / lastSample);
        y = Waveform_SampleToY(trace[startSample + index]);
        LCD_DrawLine(previousX, previousY, x, y);
        if (y + 1U <= PLOT_TOP + PLOT_HEIGHT)
        {
            LCD_Fast_DrawPoint(x, y + 1U, color);
        }
        previousX = x;
        previousY = y;
    }
}

static uint16_t Waveform_GridColor(uint16_t x, uint16_t y)
{
    uint16_t relativeX = (uint16_t)(x - PLOT_LEFT);
    uint16_t relativeY = (uint16_t)(y - PLOT_TOP);

    if (x == PLOT_LEFT || x == PLOT_LEFT + PLOT_WIDTH ||
        y == PLOT_TOP || y == PLOT_TOP + PLOT_HEIGHT ||
        y == PLOT_TOP + PLOT_HEIGHT / 2U)
    {
        return WHITE;
    }
    if ((relativeX % 54U) == 0U || (relativeY % 36U) == 0U)
    {
        return COLOR_GRID_MAIN;
    }
    if ((relativeX % 27U) == 0U || (relativeY % 18U) == 0U)
    {
        return COLOR_GRID;
    }
    return COLOR_PLOT_BG;
}

static void Waveform_RestorePoint(uint16_t x, uint16_t y)
{
    LCD_Fast_DrawPoint(x, y, Waveform_GridColor(x, y));
}

static void Waveform_RestoreLine(uint16_t x1, uint16_t y1,
                                 uint16_t x2, uint16_t y2)
{
    uint16_t count;
    int32_t xError = 0;
    int32_t yError = 0;
    int32_t deltaX = (int32_t)x2 - x1;
    int32_t deltaY = (int32_t)y2 - y1;
    int32_t incrementX;
    int32_t incrementY;
    int32_t distance;
    int32_t row = x1;
    int32_t column = y1;

    if (deltaX > 0) incrementX = 1;
    else if (deltaX == 0) incrementX = 0;
    else
    {
        incrementX = -1;
        deltaX = -deltaX;
    }
    if (deltaY > 0) incrementY = 1;
    else if (deltaY == 0) incrementY = 0;
    else
    {
        incrementY = -1;
        deltaY = -deltaY;
    }
    distance = deltaX > deltaY ? deltaX : deltaY;
    for (count = 0U; count <= (uint16_t)distance + 1U; count++)
    {
        Waveform_RestorePoint((uint16_t)row, (uint16_t)column);
        xError += deltaX;
        yError += deltaY;
        if (xError > distance)
        {
            xError -= distance;
            row += incrementX;
        }
        if (yError > distance)
        {
            yError -= distance;
            column += incrementY;
        }
    }
}

static void Waveform_RestoreTraceData(const uint16_t *trace)
{
    uint16_t lastSample = timebaseLastSample[timeZoomIndex];
    uint16_t startSample = (SAMPLE_COUNT - 1U) - lastSample;
    uint16_t index;
    uint16_t previousX = PLOT_LEFT;
    uint16_t previousY = Waveform_SampleToY(trace[startSample]);
    uint16_t x;
    uint16_t y;

    for (index = 1U; index <= lastSample; index++)
    {
        x = PLOT_LEFT + (uint16_t)((uint32_t)index * PLOT_WIDTH / lastSample);
        y = Waveform_SampleToY(trace[startSample + index]);
        Waveform_RestoreLine(previousX, previousY, x, y);
        if (y + 1U <= PLOT_TOP + PLOT_HEIGHT)
        {
            Waveform_RestorePoint(x, y + 1U);
        }
        previousX = x;
        previousY = y;
    }
}

static void Waveform_DrawMeasurements(void)
{
    uint32_t absoluteOffset = offsetMv < 0 ? (uint32_t)-offsetMv : (uint32_t)offsetMv;

    LCD_Fill(622U, 120U, 780U, 143U, COLOR_PANEL);
    LCD_Fill(622U, 170U, 780U, 193U, COLOR_PANEL);
    LCD_Fill(622U, 220U, 780U, 243U, COLOR_PANEL);
    LCD_Fill(622U, 270U, 780U, 296U, COLOR_PANEL);
    LCD_Fill(622U, 326U, 780U, 350U, COLOR_PANEL);
    Waveform_DrawUnsigned(626U, 120U, frequencyHz, "HZ", WHITE, 2U);
    Waveform_DrawUnsigned(626U, 170U, peakToPeakMv, "MV", COLOR_WAVE, 2U);
    Waveform_DrawUnsigned(626U, 220U, rmsMv, "MV", COLOR_ACCENT, 2U);
    Waveform_DrawText(626U, 276U, offsetMv < 0 ? "-" : "+", WHITE, 2U);
    Waveform_DrawUnsigned(643U, 276U, absoluteOffset, "MV", WHITE, 2U);
    Waveform_DrawUnsigned(626U, 328U, sampleRateHz, "SPS", WHITE, 2U);
}

static void Waveform_DrawCursor(void)
{
    /* 绘制暂停测量时的十字、圆形采样点及顶部时间/电压读数。 */
    uint32_t timeValue;
    int32_t voltageMv;
    uint32_t absoluteVoltage;
    uint32_t sampleIndex;
    uint64_t timeUs64;
    const char *timeUnit;

    if (cursorEnabled == 0U)
    {
        return;
    }

    POINT_COLOR = COLOR_CURSOR;
    LCD_DrawLine(cursorX, PLOT_TOP, cursorX, PLOT_TOP + PLOT_HEIGHT);
    LCD_DrawLine(PLOT_LEFT, cursorY, PLOT_LEFT + PLOT_WIDTH, cursorY);
    LCD_Draw_Circle(cursorX, cursorY, 5U);

    sampleIndex = (uint32_t)(cursorX - PLOT_LEFT) *
                  timebaseLastSample[timeZoomIndex] / PLOT_WIDTH;
    timeUs64 = ((uint64_t)sampleIndex * sampleDiv + ADC_CYCLES_US / 2U) /
               ADC_CYCLES_US;
    if (timeUs64 > 99999U)
    {
        uint64_t timeMs64 = (timeUs64 + 500U) / 1000U;
        timeValue = timeMs64 > 99999U ? 99999U : (uint32_t)timeMs64;
        timeUnit = "MS";
    }
    else
    {
        timeValue = (uint32_t)timeUs64;
        timeUnit = "US";
    }
    voltageMv = 5000 - (int32_t)(cursorY - PLOT_TOP) * 10000 / PLOT_HEIGHT;
    absoluteVoltage = voltageMv < 0 ? (uint32_t)-voltageMv : (uint32_t)voltageMv;

    LCD_Fill(360U, 8U, 608U, 33U, 0x014BU);
    Waveform_DrawText(365U, 12U, "T", COLOR_CURSOR, 1U);
    Waveform_DrawValue(378U, 9U, timeValue, 5U, timeUnit, WHITE);
    Waveform_DrawText(470U, 12U, voltageMv < 0 ? "V-" : "V+", COLOR_CURSOR, 1U);
    Waveform_DrawValue(489U, 9U, absoluteVoltage, 4U, "MV", WHITE);
}

static void Waveform_ClearCursor(void)
{
    /* 擦除游标并恢复它覆盖的网格和波形，保持界面无残影。 */
    if (cursorDrawn == 0U)
    {
        return;
    }

    POINT_COLOR = COLOR_PLOT_BG;
    LCD_DrawLine(cursorX, PLOT_TOP, cursorX, PLOT_TOP + PLOT_HEIGHT);
    LCD_DrawLine(PLOT_LEFT, cursorY, PLOT_LEFT + PLOT_WIDTH, cursorY);
    LCD_Draw_Circle(cursorX, cursorY, 5U);
    Waveform_RedrawGridLines();
    if (traceDrawn != 0U)
    {
        Waveform_DrawTraceData(samples, COLOR_WAVE);
    }

    /* 清除顶栏中的游标时间和电压读数。 */
    LCD_Fill(360U, 8U, 608U, 33U, 0x014BU);
    cursorDrawn = 0U;
    cursorEnabled = 0U;
}

static void Waveform_DrawRunState(void)
{
    /* 更新右上角 RUN/PAUSE 按钮和左侧运行状态提示。 */
    LCD_Fill(618U, 5U, 786U, 35U, running != 0U ? 0x0861U : 0x7800U);
    POINT_COLOR = running != 0U ? COLOR_ACCENT : YELLOW;
    LCD_DrawRectangle(618U, 5U, 786U, 35U);
    Waveform_DrawText(running != 0U ? 638U : 647U, 14U,
                      running != 0U ? "PAUSE" : "RUN", WHITE, 1U);
    LCD_Fill(548U, 10U, 606U, 30U, 0x014BU);
    Waveform_DrawText(552U, 14U, running != 0U ? "RUN" : "HOLD",
                      running != 0U ? GREEN : YELLOW, 1U);
}

void Waveform_Init(uint8_t touchAvailable)
{
    touchEnabled = touchAvailable;
    cursorEnabled = 0U;
    cursorDrawn = 0U;
    traceDrawn = 0U;
    running = 1U;
    timeZoomIndex = 0U;
    framePending = 0U;
    metadataPending = 0U;
    frameAvailable = 0U;
    signalFlags = 0U;
    frameSequence = 0U;
    linkTimedOut = 0U;
    lastFrameTickMs = 0U;
    frequencyHz = 0U;
    sampleDiv = 0U;
    sampleRateHz = 0U;
    peakToPeakMv = 0U;
    offsetMv = 0;
    rmsMv = 0U;
    linkBytesReceived = 0U;
    linkValidFrames = 0U;
    linkTotalErrors = 0U;
    lastMeasurementDrawTickMs = HAL_GetTick();
    measurementDirty = 0U;
    displayedSampleDiv = 0U;
    Waveform_DrawStaticLayout();
    Waveform_DrawGrid();
    Waveform_DrawRunState();
    Waveform_DrawMeasurements();
    Waveform_DrawTimebase();
    Waveform_DrawLinkState();
    Waveform_DrawLinkStats();
}

void Waveform_SubmitFrame(const uint16_t inputSamples[WAVEFORM_SAMPLE_COUNT],
                          uint32_t inputFrequencyHz, uint32_t inputSampleDiv,
                          uint32_t inputPeriodCycles,
                          uint8_t flags, uint8_t sequence)
{
    uint16_t index;
    uint16_t value;
    uint16_t firstVisibleSample =
        (SAMPLE_COUNT - 1U) - timebaseLastSample[timeZoomIndex];
    uint8_t samplesChanged = frameAvailable == 0U ? 1U : 0U;

    if (inputSamples == NULL)
    {
        return;
    }

    for (index = 0U; index < SAMPLE_COUNT; index++)
    {
        value = (flags & 0x01U) == 0U ? 512U :
                (inputSamples[index] > 1023U ? 1023U : inputSamples[index]);
        pendingSamples[index] = (uint16_t)((value << 2U) | (value >> 8U));
        if (index >= firstVisibleSample &&
            Waveform_SampleToY(pendingSamples[index]) !=
            Waveform_SampleToY(samples[index]))
        {
            samplesChanged = 1U;
        }
    }
    pendingFrequencyHz = inputFrequencyHz;
    pendingSampleDiv = inputSampleDiv;
    pendingPeriodCycles = inputPeriodCycles;
    pendingFlags = flags;
    pendingSequence = sequence;
    lastFrameTickMs = HAL_GetTick();
    framePending = samplesChanged;
    metadataPending = 1U;
}

static uint32_t Waveform_SelectFrequency(uint32_t gateFrequency,
                                         uint32_t periodCycles,
                                         uint8_t flags)
{
    if ((flags & 0x01U) == 0U)
    {
        return 0U;
    }
    if (periodCycles >= 16U && periodCycles <= ADC_CLOCK_HZ)
    {
        return (uint32_t)(((uint64_t)ADC_CLOCK_HZ + periodCycles / 2U) /
                          periodCycles);
    }
    return gateFrequency;
}

void Waveform_Update(void)
{
    uint16_t index;
    uint8_t timeoutNow;
    uint8_t timebaseChanged = 0U;
    uint8_t linkStateChanged = 0U;
    uint8_t frequencyChanged = 0U;
    uint8_t measurementStateChanged = 0U;

    timeoutNow = (frameAvailable != 0U &&
                  (HAL_GetTick() - lastFrameTickMs) > LINK_TIMEOUT_MS) ? 1U : 0U;
    if (timeoutNow != linkTimedOut)
    {
        linkTimedOut = timeoutNow;
        Waveform_DrawLinkState();
    }

    if (metadataPending != 0U)
    {
        uint8_t wasFrameAvailable = frameAvailable;
        uint8_t previousFlags = signalFlags;
        uint32_t previousSampleDiv = sampleDiv;
        uint32_t previousFrequency = frequencyHz;
        uint32_t expectedSampleDiv;
        uint32_t displayedDivDifference;
        uint32_t displayedDivTolerance;

        signalFlags = pendingFlags;
        frameSequence = pendingSequence;
        if ((signalFlags & 0x01U) == 0U)
        {
            sampleDiv = 0U;
            peakToPeakMv = 0U;
            rmsMv = 0U;
            offsetMv = 0;
        }
        else
        {
            sampleDiv = pendingSampleDiv;
            if (pendingPeriodCycles >= 64U &&
                pendingPeriodCycles <= ADC_CLOCK_HZ)
            {
                expectedSampleDiv = (pendingPeriodCycles + 32U) >> 6U;
                if (expectedSampleDiv == 0U)
                {
                    expectedSampleDiv = 1U;
                }
                sampleDiv = expectedSampleDiv;
            }
        }
        sampleRateHz = sampleDiv == 0U ? 0U : ADC_CLOCK_HZ / sampleDiv;
        frequencyHz = Waveform_SelectFrequency(pendingFrequencyHz,
                                               pendingPeriodCycles, signalFlags);
        frameAvailable = 1U;
        linkTimedOut = 0U;
        metadataPending = 0U;
        displayedDivDifference = sampleDiv > displayedSampleDiv ?
                                 sampleDiv - displayedSampleDiv :
                                 displayedSampleDiv - sampleDiv;
        displayedDivTolerance = displayedSampleDiv / 50U;
        if (displayedDivTolerance == 0U)
        {
            displayedDivTolerance = 1U;
        }
        timebaseChanged = (wasFrameAvailable == 0U ||
                           ((signalFlags ^ previousFlags) & 0x01U) != 0U ||
                           displayedDivDifference > displayedDivTolerance) ?
                          1U : 0U;
        frequencyChanged = (wasFrameAvailable == 0U ||
                            frequencyHz != previousFrequency) ? 1U : 0U;
        measurementStateChanged = (wasFrameAvailable == 0U ||
                                   ((signalFlags ^ previousFlags) & 0x01U) != 0U) ?
                                  1U : 0U;
        if (frequencyChanged != 0U || sampleDiv != previousSampleDiv ||
            timebaseChanged != 0U ||
            measurementStateChanged != 0U)
        {
            measurementDirty = 1U;
        }
        linkStateChanged = (wasFrameAvailable == 0U ||
                            signalFlags != previousFlags ||
                            (pendingSequence & 0x07U) == 0U) ? 1U : 0U;
        if (framePending == 0U)
        {
            if (timebaseChanged != 0U)
            {
                Waveform_DrawTimebase();
            }
            if (linkStateChanged != 0U)
            {
                Waveform_DrawLinkState();
            }
        }
    }

    if ((running == 0U || framePending == 0U) && measurementDirty != 0U &&
        (measurementStateChanged != 0U ||
         (HAL_GetTick() - lastMeasurementDrawTickMs) >=
         MEASUREMENT_REFRESH_MS))
    {
        Waveform_DrawMeasurements();
        lastMeasurementDrawTickMs = HAL_GetTick();
        measurementDirty = 0U;
    }

    if (running == 0U || framePending == 0U)
    {
        return;
    }

    Waveform_ClearCursor();
    if (traceDrawn != 0U)
    {
        Waveform_RestoreTraceData(previousSamples);
    }

    for (index = 0U; index < SAMPLE_COUNT; index++)
    {
        samples[index] = pendingSamples[index];
    }
    framePending = 0U;

    if ((signalFlags & 0x01U) != 0U)
    {
        Waveform_ComputeMeasurements();
    }
    measurementDirty = 1U;
    Waveform_DrawTraceData(samples, COLOR_WAVE);
    if (measurementStateChanged != 0U ||
        (HAL_GetTick() - lastMeasurementDrawTickMs) >= MEASUREMENT_REFRESH_MS)
    {
        Waveform_DrawMeasurements();
        lastMeasurementDrawTickMs = HAL_GetTick();
        measurementDirty = 0U;
    }
    if (timebaseChanged != 0U)
    {
        Waveform_DrawTimebase();
    }
    if (linkStateChanged != 0U)
    {
        Waveform_DrawLinkState();
    }
    for (index = 0U; index < SAMPLE_COUNT; index++)
    {
        previousSamples[index] = samples[index];
    }
    traceDrawn = 1U;
}

void Waveform_SetLinkStats(uint32_t bytesReceived, uint32_t validFrames,
                           uint32_t totalErrors)
{
    if (bytesReceived == linkBytesReceived &&
        validFrames == linkValidFrames && totalErrors == linkTotalErrors)
    {
        return;
    }
    linkBytesReceived = bytesReceived;
    linkValidFrames = validFrames;
    linkTotalErrors = totalErrors;
    Waveform_DrawLinkStats();
}

void Waveform_SetTouchAvailable(uint8_t touchAvailable)
{
    if (touchEnabled == touchAvailable)
    {
        return;
    }
    touchEnabled = touchAvailable;
    LCD_Fill(622U, 390U, 782U, 416U, COLOR_PANEL);
    Waveform_DrawText(626U, 394U,
                      touchEnabled != 0U ? "TOUCH READY" : "NOT FOUND",
                      touchEnabled != 0U ? GREEN : YELLOW, 1U);
}

void Waveform_HandleTouch(uint16_t x, uint16_t y)
{
    /* 处理按钮、波形区域和其他区域的点击状态转换。 */
    uint8_t requestedZoom;

    if (x >= 618U && x <= 786U && y <= 42U)
    {
        running ^= 1U;
        if (running != 0U)
        {
            Waveform_ClearCursor();
        }
        Waveform_DrawRunState();
        return;
    }
    if (y >= 423U && y <= 479U &&
        ((x >= PLOT_LEFT && x <= PLOT_LEFT + 90U) ||
         (x >= PLOT_LEFT + PLOT_WIDTH - 90U && x <= PLOT_LEFT + PLOT_WIDTH)))
    {
        requestedZoom = timeZoomIndex;
        if (x < PLOT_LEFT + PLOT_WIDTH / 2U)
        {
            if (requestedZoom > 0U)
            {
                requestedZoom--;
            }
        }
        else if (requestedZoom + 1U < TIME_ZOOM_LEVELS)
        {
            requestedZoom++;
        }
        if (requestedZoom == timeZoomIndex)
        {
            return;
        }
        Waveform_ClearCursor();
        if (traceDrawn != 0U)
        {
            Waveform_RestoreTraceData(samples);
        }
        timeZoomIndex = requestedZoom;
        if (traceDrawn != 0U)
        {
            Waveform_DrawTraceData(samples, COLOR_WAVE);
        }
        Waveform_DrawTimebase();
        return;
    }
    if (cursorDrawn != 0U)
    {
        /* 已有测量点时，下一次点击只关闭游标，不立即移动游标。 */
        Waveform_ClearCursor();
        return;
    }
    if (x < PLOT_LEFT || x > PLOT_LEFT + PLOT_WIDTH ||
        y < PLOT_TOP || y > PLOT_TOP + PLOT_HEIGHT)
    {
        /* 暂停时点击其他区域，仅取消测量点，不改变暂停状态。 */
        Waveform_ClearCursor();
        return;
    }
    cursorEnabled = 1U;
    cursorDrawn = 1U;
    cursorX = x;
    cursorY = y;
    running = 0U;
    Waveform_DrawRunState();
    Waveform_DrawCursor();
}

void Waveform_ShowTouch(uint16_t x, uint16_t y)
{
    /* 显示最近一次原始逻辑坐标，用于现场确认触摸方向和响应。 */
    char xText[4];
    char yText[4];

    lastTouchX = x;
    lastTouchY = y;
    Waveform_UintToText(lastTouchX, xText, 3U);
    Waveform_UintToText(lastTouchY, yText, 3U);
    LCD_Fill(622U, 390U, 782U, 416U, COLOR_PANEL);
    Waveform_DrawText(626U, 394U, "X", GREEN, 1U);
    Waveform_DrawText(638U, 394U, xText, WHITE, 1U);
    Waveform_DrawText(668U, 394U, "Y", GREEN, 1U);
    Waveform_DrawText(680U, 394U, yText, WHITE, 1U);
}
