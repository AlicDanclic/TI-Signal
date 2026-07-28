#include "touch.h"

#include "main.h"

/*
 * 触摸驱动使用 PH6/PI3 软件 I2C，兼容官方 4.3 寸模块的 GT9147 和 OTT2001A。
 * 控制器原始坐标在 Touch_Read 中转换为 LCD 横屏坐标，应用不直接操作寄存器。
 */

/*
 * ALIENTEK 4.3 寸 TFTLCD 的电容触摸接口为软件 I2C：
 * PH6=SCL，PI3=SDA，PI8=复位，PH7=中断。官方模块存在 GT9147 和
 * OTT2001A 两种触摸控制器，本驱动会在启动时依次探测。
 */
#define GT_WRITE_ADDRESS 0x28U
#define GT_READ_ADDRESS  0x29U
#define GT_PRODUCT_ID    0x8140U
#define GT_RESOLUTION    0x8048U
#define GT_STATUS        0x814EU
#define GT_POINT_ONE     0x8150U
#define GT_ALT_WRITE_ADDRESS 0xBAU
#define GT_ALT_READ_ADDRESS  0xBBU

#define FT_WRITE_ADDRESS 0x70U
#define FT_READ_ADDRESS  0x71U
#define FT_TOUCH_COUNT   0x02U
#define FT_POINT_ONE     0x03U
#define FT_CHIP_ID       0xA3U

#define OTT_WRITE_ADDRESS 0xB2U
#define OTT_READ_ADDRESS  0xB3U
#define OTT_STATUS        0x0000U
#define OTT_POINT_ONE     0x0100U
#define OTT_CONTROL       0x0D00U

#define TOUCH_FAILURE_LIMIT 8U
#define TOUCH_STALE_HOLD_MS 30U
#define TOUCH_HARD_RESET_RETRIES 10U

static TouchController controller = TOUCH_CONTROLLER_NONE;
static uint8_t gtWriteAddress = GT_WRITE_ADDRESS;
static uint8_t gtReadAddress = GT_READ_ADDRESS;
static uint16_t touchMaximumX = 800U;
static uint16_t touchMaximumY = 480U;
static uint16_t lastX;
static uint16_t lastY;
static uint8_t touchActive;
static uint32_t lastTouchDataTick;
static uint8_t communicationFailures;
static uint8_t reconnectAttempts;

static void Touch_Delay(void)
{
    /* 软件 I2C 半周期约 4 us，降低边沿速度以提高长排线可靠性。 */
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = SystemCoreClock / 250000U;
    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

static void Touch_SdaOutput(void)
{
    /* PI3 切换为开漏输出，发送 I2C 地址、寄存器和 ACK。 */
    GPIOI->MODER = (GPIOI->MODER & ~(3UL << (3U * 2U))) | (1UL << (3U * 2U));
}

static void Touch_SdaInput(void)
{
    /* PI3 切换为输入，释放总线并读取从机数据。 */
    GPIOI->MODER &= ~(3UL << (3U * 2U));
}

static void Touch_SetScl(GPIO_PinState state)
{
    uint32_t start;

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_6, state);
    if (state == GPIO_PIN_SET)
    {
        start = DWT->CYCCNT;
        while (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_6) == GPIO_PIN_RESET &&
               (DWT->CYCCNT - start) < SystemCoreClock / 50000U)
        {
        }
    }
}

static void Touch_SetSda(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_3, state);
}

static void Touch_Start(void)
{
    /* I2C 起始条件：SCL 为高时 SDA 从高变低。 */
    Touch_SdaOutput();
    Touch_SetSda(GPIO_PIN_SET);
    Touch_SetScl(GPIO_PIN_SET);
    Touch_Delay();
    Touch_SetSda(GPIO_PIN_RESET);
    Touch_Delay();
    Touch_SetScl(GPIO_PIN_RESET);
}

static void Touch_Stop(void)
{
    /* I2C 停止条件：SCL 为高时 SDA 从低变高。 */
    Touch_SdaOutput();
    Touch_SetSda(GPIO_PIN_RESET);
    Touch_Delay();
    Touch_SetScl(GPIO_PIN_SET);
    Touch_Delay();
    Touch_SetSda(GPIO_PIN_SET);
}

static void Touch_RecoverBus(void)
{
    uint8_t pulse;

    Touch_SdaInput();
    for (pulse = 0U; pulse < 9U; pulse++)
    {
        Touch_SetScl(GPIO_PIN_RESET);
        Touch_Delay();
        Touch_SetScl(GPIO_PIN_SET);
        Touch_Delay();
    }
    Touch_Stop();
}

static void Touch_CommunicationFailed(void)
{
    if (communicationFailures < 0xFFU)
    {
        communicationFailures++;
    }
    if (communicationFailures == 3U)
    {
        Touch_RecoverBus();
    }
    if (communicationFailures >= TOUCH_FAILURE_LIMIT)
    {
        controller = TOUCH_CONTROLLER_NONE;
        touchActive = 0U;
    }
}

static uint8_t Touch_HoldLastPoint(uint16_t *x, uint16_t *y)
{
    if (touchActive != 0U &&
        (HAL_GetTick() - lastTouchDataTick) <= TOUCH_STALE_HOLD_MS)
    {
        *x = lastX;
        *y = lastY;
        return 1U;
    }
    return 0U;
}

static uint8_t Touch_WriteByte(uint8_t value)
{
    /* MSB 优先发送一个字节，并返回 1 表示未收到 ACK。 */
    uint8_t bit;
    uint8_t nack;

    Touch_SdaOutput();
    for (bit = 0U; bit < 8U; bit++)
    {
        Touch_SetSda((value & 0x80U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        Touch_Delay();
        Touch_SetScl(GPIO_PIN_SET);
        Touch_Delay();
        Touch_SetScl(GPIO_PIN_RESET);
        value <<= 1U;
    }

    Touch_SetSda(GPIO_PIN_SET);
    Touch_SdaInput();
    Touch_Delay();
    Touch_SetScl(GPIO_PIN_SET);
    Touch_Delay();
    nack = HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_3) == GPIO_PIN_SET;
    Touch_SetScl(GPIO_PIN_RESET);
    Touch_Delay();
    Touch_SdaOutput();
    return nack;
}

static uint8_t Touch_ReadByte(uint8_t acknowledge)
{
    /* 读取一个字节；除最后一个字节外，主机发送 ACK 请求继续读取。 */
    uint8_t bit;
    uint8_t value = 0U;

    Touch_SdaInput();
    for (bit = 0U; bit < 8U; bit++)
    {
        value <<= 1U;
        Touch_SetScl(GPIO_PIN_SET);
        Touch_Delay();
        if (HAL_GPIO_ReadPin(GPIOI, GPIO_PIN_3) == GPIO_PIN_SET)
        {
            value++;
        }
        Touch_SetScl(GPIO_PIN_RESET);
        Touch_Delay();
    }

    Touch_SdaOutput();
    Touch_SetSda(acknowledge != 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
    Touch_Delay();
    Touch_SetScl(GPIO_PIN_SET);
    Touch_Delay();
    Touch_SetScl(GPIO_PIN_RESET);
    Touch_Delay();
    Touch_SetSda(GPIO_PIN_SET);
    return value;
}

static uint8_t Touch_ReadRegisters(uint8_t writeAddress, uint8_t readAddress,
                                   uint16_t registerAddress, uint8_t *data,
                                   uint8_t length)
{
    /* 按“写寄存器地址 + 重启 + 读数据”流程读取连续寄存器。 */
    uint8_t index;

    Touch_Start();
    if (Touch_WriteByte(writeAddress) != 0U ||
        Touch_WriteByte((uint8_t)(registerAddress >> 8U)) != 0U ||
        Touch_WriteByte((uint8_t)registerAddress) != 0U)
    {
        Touch_Stop();
        return 0U;
    }

    Touch_Start();
    if (Touch_WriteByte(readAddress) != 0U)
    {
        Touch_Stop();
        return 0U;
    }
    for (index = 0U; index < length; index++)
    {
        data[index] = Touch_ReadByte(index + 1U < length);
    }
    Touch_Stop();
    return 1U;
}

static uint8_t Touch_WriteRegister(uint8_t address, uint16_t registerAddress,
                                   uint8_t value)
{
    /* 向指定控制器寄存器写入一个字节。 */
    Touch_Start();
    if (Touch_WriteByte(address) != 0U ||
        Touch_WriteByte((uint8_t)(registerAddress >> 8U)) != 0U ||
        Touch_WriteByte((uint8_t)registerAddress) != 0U ||
        Touch_WriteByte(value) != 0U)
    {
        Touch_Stop();
        return 0U;
    }
    Touch_Stop();
    return 1U;
}

static uint8_t Touch_ReadRegisters8(uint8_t writeAddress, uint8_t readAddress,
                                    uint8_t registerAddress, uint8_t *data,
                                    uint8_t length)
{
    uint8_t index;

    Touch_Start();
    if (Touch_WriteByte(writeAddress) != 0U ||
        Touch_WriteByte(registerAddress) != 0U)
    {
        Touch_Stop();
        return 0U;
    }
    Touch_Start();
    if (Touch_WriteByte(readAddress) != 0U)
    {
        Touch_Stop();
        return 0U;
    }
    for (index = 0U; index < length; index++)
    {
        data[index] = Touch_ReadByte(index + 1U < length);
    }
    Touch_Stop();
    return 1U;
}

static void Touch_ResetController(uint8_t interruptHigh)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &gpio);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7,
                      interruptHigh != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(10U);
    HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(50U);

    gpio.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(GPIOH, &gpio);
}

static uint8_t Touch_ProbeGt(uint8_t writeAddress, uint8_t readAddress)
{
    uint8_t productId[4] = {0};
    uint8_t resolution[4] = {0};

    if (Touch_ReadRegisters(writeAddress, readAddress, GT_PRODUCT_ID,
                            productId, 4U) == 0U ||
        productId[0] < '0' || productId[0] > '9')
    {
        return 0U;
    }

    gtWriteAddress = writeAddress;
    gtReadAddress = readAddress;
    if (Touch_ReadRegisters(writeAddress, readAddress, GT_RESOLUTION,
                            resolution, 4U) != 0U)
    {
        uint16_t maximumX = (uint16_t)(((uint16_t)resolution[1] << 8U) |
                                       resolution[0]);
        uint16_t maximumY = (uint16_t)(((uint16_t)resolution[3] << 8U) |
                                       resolution[2]);
        if (maximumX >= 320U && maximumX <= 2048U &&
            maximumY >= 240U && maximumY <= 2048U)
        {
            touchMaximumX = maximumX;
            touchMaximumY = maximumY;
        }
    }
    return 1U;
}

static uint16_t Touch_Scale(uint16_t value, uint16_t sourceMaximum,
                            uint16_t destinationMaximum)
{
    uint32_t scaled;

    if (sourceMaximum == 0U)
    {
        return 0U;
    }
    scaled = (uint32_t)value * destinationMaximum / sourceMaximum;
    return scaled >= destinationMaximum ? destinationMaximum - 1U : (uint16_t)scaled;
}

uint8_t Touch_Init(void)
{
    /* 初始化 I2C、复位脚和中断脚，并依次识别 GT9147/OTT2001A。 */
    GPIO_InitTypeDef gpio = {0};
    uint8_t control = 0U;
    uint8_t chipId = 0U;
    uint8_t attempt;

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &gpio);
    gpio.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOI, &gpio);

    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOI, &gpio);
    gpio.Pin = GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &gpio);

    Touch_SetScl(GPIO_PIN_SET);
    Touch_SetSda(GPIO_PIN_SET);
    Touch_RecoverBus();
    controller = TOUCH_CONTROLLER_NONE;
    touchActive = 0U;
    communicationFailures = 0U;
    reconnectAttempts = 0U;

    /* INT=0/1 在复位期间分别选择 Goodix 的 0x14/0x5D 七位地址。 */
    for (attempt = 0U; attempt < 2U; attempt++)
    {
        Touch_ResetController(0U);
        if (Touch_ProbeGt(GT_WRITE_ADDRESS, GT_READ_ADDRESS) != 0U)
        {
            controller = TOUCH_CONTROLLER_GT9147;
            return 1U;
        }
        Touch_ResetController(1U);
        if (Touch_ProbeGt(GT_ALT_WRITE_ADDRESS, GT_ALT_READ_ADDRESS) != 0U)
        {
            controller = TOUCH_CONTROLLER_GT9147;
            return 1U;
        }
    }

    Touch_ResetController(0U);
    for (attempt = 0U; attempt < 3U; attempt++)
    {
        if (Touch_ReadRegisters8(FT_WRITE_ADDRESS, FT_READ_ADDRESS,
                                 FT_CHIP_ID, &chipId, 1U) != 0U &&
            chipId != 0x00U && chipId != 0xFFU)
        {
            touchMaximumX = 800U;
            touchMaximumY = 480U;
            controller = TOUCH_CONTROLLER_FT5X06;
            return 1U;
        }
        HAL_Delay(20U);
    }

    for (attempt = 0U; attempt < 3U; attempt++)
    {
        if (Touch_ReadRegisters(OTT_WRITE_ADDRESS, OTT_READ_ADDRESS,
                                OTT_CONTROL, &control, 1U) != 0U)
        {
            Touch_WriteRegister(OTT_WRITE_ADDRESS, OTT_CONTROL, 0x80U);
            controller = TOUCH_CONTROLLER_OTT2001A;
            return 1U;
        }
        HAL_Delay(20U);
    }

    controller = TOUCH_CONTROLLER_NONE;
    return 0U;
}

uint8_t Touch_Reconnect(void)
{
    uint8_t chipId = 0U;
    uint8_t control = 0U;

    if (controller != TOUCH_CONTROLLER_NONE)
    {
        return 1U;
    }

    Touch_RecoverBus();
    if (Touch_ProbeGt(GT_WRITE_ADDRESS, GT_READ_ADDRESS) != 0U ||
        Touch_ProbeGt(GT_ALT_WRITE_ADDRESS, GT_ALT_READ_ADDRESS) != 0U)
    {
        controller = TOUCH_CONTROLLER_GT9147;
    }
    else if (Touch_ReadRegisters8(FT_WRITE_ADDRESS, FT_READ_ADDRESS,
                                  FT_CHIP_ID, &chipId, 1U) != 0U &&
             chipId != 0x00U && chipId != 0xFFU)
    {
        touchMaximumX = 800U;
        touchMaximumY = 480U;
        controller = TOUCH_CONTROLLER_FT5X06;
    }
    else if (Touch_ReadRegisters(OTT_WRITE_ADDRESS, OTT_READ_ADDRESS,
                                 OTT_CONTROL, &control, 1U) != 0U)
    {
        controller = TOUCH_CONTROLLER_OTT2001A;
    }

    if (controller != TOUCH_CONTROLLER_NONE)
    {
        communicationFailures = 0U;
        reconnectAttempts = 0U;
        return 1U;
    }

    reconnectAttempts++;
    if ((reconnectAttempts % TOUCH_HARD_RESET_RETRIES) == 0U)
    {
        Touch_ResetController((reconnectAttempts /
                               TOUCH_HARD_RESET_RETRIES) & 0x01U);
    }
    return 0U;
}

uint8_t Touch_Read(uint16_t *x, uint16_t *y)
{
    /* 轮询第一个触点；释放状态或通信失败时返回 0。 */
    uint8_t status;
    uint8_t point[4];
    uint16_t rawX;
    uint16_t rawY;

    if (x == NULL || y == NULL)
    {
        return 0U;
    }

    if (controller == TOUCH_CONTROLLER_GT9147)
    {
        if (Touch_ReadRegisters(gtWriteAddress, gtReadAddress,
                                GT_STATUS, &status, 1U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        if ((status & 0x80U) == 0U)
        {
            communicationFailures = 0U;
            if (touchActive != 0U &&
                (HAL_GetTick() - lastTouchDataTick) <= TOUCH_STALE_HOLD_MS)
            {
                *x = lastX;
                *y = lastY;
                return 1U;
            }
            touchActive = 0U;
            return 0U;
        }
        if ((status & 0x0FU) == 0U)
        {
            if (Touch_WriteRegister(gtWriteAddress, GT_STATUS, 0U) == 0U)
            {
                Touch_CommunicationFailed();
                return Touch_HoldLastPoint(x, y);
            }
            communicationFailures = 0U;
            touchActive = 0U;
            return 0U;
        }
        if (Touch_ReadRegisters(gtWriteAddress, gtReadAddress,
                                GT_POINT_ONE, point, 4U) == 0U)
        {
            Touch_WriteRegister(gtWriteAddress, GT_STATUS, 0U);
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        if (Touch_WriteRegister(gtWriteAddress, GT_STATUS, 0U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        communicationFailures = 0U;
        rawX = (uint16_t)(((uint16_t)point[1] << 8U) | point[0]);
        rawY = (uint16_t)(((uint16_t)point[3] << 8U) | point[2]);
        if (touchMaximumX >= touchMaximumY)
        {
            *x = Touch_Scale(rawX, touchMaximumX, 800U);
            *y = Touch_Scale(rawY, touchMaximumY, 480U);
        }
        else
        {
            *x = (uint16_t)(799U - Touch_Scale(rawY, touchMaximumY, 800U));
            *y = Touch_Scale(rawX, touchMaximumX, 480U);
        }
        lastX = *x;
        lastY = *y;
        touchActive = 1U;
        lastTouchDataTick = HAL_GetTick();
    }
    else if (controller == TOUCH_CONTROLLER_OTT2001A)
    {
        if (Touch_ReadRegisters(OTT_WRITE_ADDRESS, OTT_READ_ADDRESS,
                                OTT_STATUS, &status, 1U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        if ((status & 0x1FU) == 0U)
        {
            communicationFailures = 0U;
            touchActive = 0U;
            return 0U;
        }
        if (Touch_ReadRegisters(OTT_WRITE_ADDRESS, OTT_READ_ADDRESS,
                                OTT_POINT_ONE, point, 4U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        communicationFailures = 0U;
        rawX = (uint16_t)(((uint16_t)point[0] << 8U) | point[1]);
        rawY = (uint16_t)(((uint16_t)point[2] << 8U) | point[3]);
        *x = rawX < 2700U ? (uint16_t)(799U - ((uint32_t)rawX * 800U / 2700U)) : 0U;
        *y = (uint16_t)((uint32_t)rawY * 480U / 1500U);
        lastX = *x;
        lastY = *y;
        touchActive = 1U;
        lastTouchDataTick = HAL_GetTick();
    }
    else if (controller == TOUCH_CONTROLLER_FT5X06)
    {
        if (Touch_ReadRegisters8(FT_WRITE_ADDRESS, FT_READ_ADDRESS,
                                 FT_TOUCH_COUNT, &status, 1U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        if ((status & 0x0FU) == 0U)
        {
            communicationFailures = 0U;
            touchActive = 0U;
            return 0U;
        }
        if (Touch_ReadRegisters8(FT_WRITE_ADDRESS, FT_READ_ADDRESS,
                                 FT_POINT_ONE, point, 4U) == 0U)
        {
            Touch_CommunicationFailed();
            return Touch_HoldLastPoint(x, y);
        }
        communicationFailures = 0U;
        rawX = (uint16_t)((((uint16_t)point[0] & 0x0FU) << 8U) | point[1]);
        rawY = (uint16_t)((((uint16_t)point[2] & 0x0FU) << 8U) | point[3]);
        *x = Touch_Scale(rawX, touchMaximumX, 800U);
        *y = Touch_Scale(rawY, touchMaximumY, 480U);
        lastX = *x;
        lastY = *y;
        touchActive = 1U;
        lastTouchDataTick = HAL_GetTick();
    }
    else
    {
        return 0U;
    }

    return *x < 800U && *y < 480U;
}

TouchController Touch_GetController(void)
{
    /* 返回启动时探测到的控制器型号，供调试或状态显示使用。 */
    return controller;
}
