# 移植过程
1. 新建components文件夹，并在最外层CMakeLists.txt文件添加如下代码
```
set(EXTRA_COMPONENT_DIRS ./components)
```
2. components目录下下载lvgl源码
```bash 
git clone --recursive https://github.com/lvgl/lvgl.git
```
3. components目录下新建bsp文件，用于存放显示和触摸驱动
```c fold title:st7789_driver.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "st7789_driver.h"

#define LCD_SPI_HOST    SPI2_HOST

static const char* TAG = "st7789";

//lcd操作句柄
static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;

//刷新完成回调函数
static lcd_flush_done_cb    s_flush_done_cb = NULL;

//背光GPIO
static gpio_num_t   s_bl_gpio = -1;

static bool notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    if(s_flush_done_cb)
        s_flush_done_cb(user_ctx);
    return false;
}


/** st7789初始化
 * @param st7789_cfg_t  接口参数
 * @return 成功或失败
*/
esp_err_t st7789_driver_hw_init(st7789_cfg_t* cfg)
{
    //初始化SPI
    spi_bus_config_t buscfg = {
        .sclk_io_num = cfg->clk,        //SCLK引脚
        .mosi_io_num = cfg->mosi,       //MOSI引脚
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .flags = SPICOMMON_BUSFLAG_MASTER , //SPI主模式
        .max_transfer_sz = cfg->width * 40 * sizeof(uint16_t),  //DMA单次传输最大字节，最大32768
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    s_flush_done_cb = cfg->done_cb; //设置刷新完成回调函数

    s_bl_gpio = cfg->bl;    //设置背光GPIO
    //初始化GPIO(BL)
    gpio_config_t bl_gpio_cfg = 
    {
        .pull_up_en = GPIO_PULLUP_DISABLE,          //禁止上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,      //禁止下拉
        .mode = GPIO_MODE_OUTPUT,                   //输出模式
        .intr_type = GPIO_INTR_DISABLE,             //禁止中断
        .pin_bit_mask = (1<<cfg->bl)                //GPIO脚
    };
    gpio_config(&bl_gpio_cfg);


    //初始化复位脚
    if(cfg->rst > 0)
    {
        gpio_config_t rst_gpio_cfg = 
        {
            .pull_up_en = GPIO_PULLUP_DISABLE,          //禁止上拉
            .pull_down_en = GPIO_PULLDOWN_DISABLE,      //禁止下拉
            .mode = GPIO_MODE_OUTPUT,                   //输出模式
            .intr_type = GPIO_INTR_DISABLE,             //禁止中断
            .pin_bit_mask = (1<<cfg->rst)                //GPIO脚
        };
        gpio_config(&rst_gpio_cfg);
    }

    //创建基于spi的lcd操作句柄
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = cfg->dc,         //DC引脚
        .cs_gpio_num = cfg->cs,         //CS引脚
        .pclk_hz = cfg->spi_fre,        //SPI时钟频率
        .lcd_cmd_bits = 8,              //命令长度
        .lcd_param_bits = 8,            //参数长度
        .spi_mode = 0,                  //使用SPI0模式
        .trans_queue_depth = 10,        //表示可以缓存的spi传输事务队列深度
        .on_color_trans_done = notify_flush_ready,   //刷新完成回调函数
        .user_ctx = cfg->cb_param,                                    //回调函数参数
        .flags = {    // 以下为 SPI 时序的相关参数，需根据 LCD 驱动 IC 的数据手册以及硬件的配置确定
            .sio_mode = 0,    // 通过一根数据线（MOSI）读写数据，0: Interface I 型，1: Interface II 型
        },
    };
    // Attach the LCD to the SPI bus
    ESP_LOGI(TAG,"create esp_lcd_new_panel");
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &lcd_io_handle));
    
    //硬件复位
    if(cfg->rst > 0)
    {
        gpio_set_level(cfg->rst,0);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(cfg->rst,1);
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /*向LCD写入初始化命令*/
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_SWRESET,NULL,0);    //软件复位
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_SLPOUT,NULL,0);     //退出休眠模式
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_COLMOD,(uint8_t[]) {0x55,}, 1);  //选择RGB数据格式，0x55:RGB565,0x66:RGB666
    esp_lcd_panel_io_tx_param(lcd_io_handle, 0xb0, (uint8_t[]) {0x00, 0xF0},2);

    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_INVON,NULL,0);     //颜色翻转
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_NORON,NULL,0);     //普通显示模式
    uint8_t spin_type = 0;
    switch(cfg->spin)
    {
        case 0:
            spin_type = 0x00;   //不旋转
            break;
        case 1:
            spin_type = 0x60;   //顺时针90
            break;
        case 2:
            spin_type = 0xC0;   //180
            break;
        case 3:
            spin_type = 0xA0;   //顺时针270,（逆时针90）
            break;
        default:break;
    }
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_MADCTL,(uint8_t[]) {spin_type,}, 1);   //屏旋转方向
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_DISPON,NULL,0);    //打开显示
    vTaskDelay(pdMS_TO_TICKS(300));
    return ESP_OK;
}

/** st7789写入显示数据
 * @param x1,x2,y1,y2:显示区域
 * @return 无
*/
void st7789_flush(int x1,int x2,int y1,int y2,void *data)
{
    // define an area of frame memory where MCU can access
    if(x2 <= x1 || y2 <= y1)
    {
        if(s_flush_done_cb)
            s_flush_done_cb(NULL);
        return;
    }
    esp_lcd_panel_io_tx_param(lcd_io_handle, LCD_CMD_CASET, (uint8_t[]) {
        (x1 >> 8) & 0xFF,
        x1 & 0xFF,
        ((x2 - 1) >> 8) & 0xFF,
        (x2 - 1) & 0xFF,
    }, 4);
    esp_lcd_panel_io_tx_param(lcd_io_handle, LCD_CMD_RASET, (uint8_t[]) {
        (y1 >> 8) & 0xFF,
        y1 & 0xFF,
        ((y2 - 1) >> 8) & 0xFF,
        (y2 - 1) & 0xFF,
    }, 4);
    // transfer frame buffer
    size_t len = (x2 - x1) * (y2 - y1) * 2;
    esp_lcd_panel_io_tx_color(lcd_io_handle, LCD_CMD_RAMWR, data, len);
    return ;
}

/** 控制背光
 * @param enable 是否使能背光
 * @return 无
*/
void st7789_lcd_backlight(bool enable)
{
    if(enable)
    {
        gpio_set_level(s_bl_gpio,1);
    }
    else
    {
        gpio_set_level(s_bl_gpio,0);
    }
}

```
```c fold title:st7789_driver.h 
#ifndef _ST7789_DRIVER_H_
#define _ST7789_DRIVER_H_
#include "driver/gpio.h"
#include "esp_err.h"

typedef void(*lcd_flush_done_cb)(void* param);

typedef struct
{
    gpio_num_t  mosi;       //数据
    gpio_num_t  clk;        //时钟
    gpio_num_t  cs;         //片选
    gpio_num_t  dc;         //命令
    gpio_num_t  rst;        //复位
    gpio_num_t  bl;         //背光
    uint32_t    spi_fre;    //spi总线速率
    uint16_t    width;      //宽
    uint16_t    height;     //长
    uint8_t     spin;       //选择方向(0不旋转，1顺时针旋转90, 2旋转180，3顺时针旋转270)
    lcd_flush_done_cb   done_cb;    //数据传输完成回调函数
    void*       cb_param;   //回调函数参数
}st7789_cfg_t;


/** st7789初始化
 * @param st7789_cfg_t  接口参数
 * @return 成功或失败
*/
esp_err_t st7789_driver_hw_init(st7789_cfg_t* cfg);

/** st7789写入显示数据
 * @param x1,x2,y1,y2:显示区域
 * @return 无
*/
void st7789_flush(int x1,int x2,int y1,int y2,void *data);

/** 控制背光
 * @param enable 是否使能背光
 * @return 无
*/
void st7789_lcd_backlight(bool enable);

#endif

```
4. main目录下创建 lv_port.c和lv_port.h，这个文件则是具体的移植文件
```c fold title:lv_port.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_log.h"
#include "st7789_driver.h"
#include "esp_timer.h"

#define TAG "lv_port"

// 引脚定义
#define PIN_LCD_SCLK    GPIO_NUM_5
#define PIN_LCD_MOSI    GPIO_NUM_6
#define PIN_LCD_CS      GPIO_NUM_16
#define PIN_LCD_DC      GPIO_NUM_15
#define PIN_LCD_RST     GPIO_NUM_7
#define PIN_LCD_BL      GPIO_NUM_17

// 屏幕参数
#define LCD_PIXEL_CLOCK_HZ   (40 * 1000 * 1000) // 40MHz SPI时钟
#define LCD_H_RES            240
#define LCD_V_RES            320
#define LCD_BITS_PER_PIXEL   16 // RGB565

/*
1.初始化和注册LVGL显示驱动
2.初始化和注册LVGL触摸驱动
3.初始化ST7789驱动
4.初始化CST816T硬件接口
5.提供一个定时器
*/

static lv_disp_drv_t disp_drv;

void lv_flush_done_cb(void* param) {
    lv_disp_flush_ready(&disp_drv);
}

void display_flush(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
    st7789_flush(area->x1, area->x2 + 1, area->y1, area->y2 + 1, color_p);
}

void lv_disp_init(void) {
    static lv_disp_draw_buf_t draw_buf;
    const size_t disp_buf_size = LCD_H_RES * LCD_V_RES / 6; // 定义缓冲区大小 （1/4到1/6屏幕大小）
    lv_color_t *disp1 = heap_caps_malloc(disp_buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *disp2 = heap_caps_malloc(disp_buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!disp1 || !disp2) {
        ESP_LOGE(TAG, "Failed to allocate display buffers");
        return;
    }
    lv_disp_draw_buf_init(&draw_buf, disp1, disp2, disp_buf_size);
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = display_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

void st778_hw_init(void) {
    st7789_cfg_t lcd_config = {
        .mosi = PIN_LCD_MOSI,
        .clk = PIN_LCD_SCLK,
        .cs = PIN_LCD_CS,
        .dc = PIN_LCD_DC,
        .rst = PIN_LCD_RST,
        .bl = PIN_LCD_BL,
        .spi_fre = LCD_PIXEL_CLOCK_HZ,
        .width = LCD_V_RES,
        .height = LCD_H_RES,
        .spin = 0,
        .done_cb = lv_flush_done_cb,
        .cb_param = &disp_drv,
    };
    st7789_driver_hw_init(&lcd_config);
}


void lv_timer_cb(void* arg) {
    uint32_t tic_interval = *((uint32_t*)arg); 
    lv_tick_inc(tic_interval);
}

void lv_tick_init(void) {
    static uint32_t tic_interval = 5;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_inc,
        .arg = &tic_interval,
        .name = "lvgl_tick",
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, tic_interval * 1000)); // 1ms
}
void lv_port_init(void) {
    lv_init();
    st778_hw_init();
    lv_disp_init();
    lv_tick_init();
}
```
```c fold title:lv_port.h
# ifndef _LV_PORT_H
#define _LV_PORT_H
void lv_port_init(void);
#endif
```
5. idf.py menuconfig
	1. Color setting  ==swap the 2 bytes of RGB 565 color==
	2. Memory setting ==If true use custom malloc/free ==
	3. Demos ==启用第一个demo==

# 其他注意事项
1. 缓冲去大小一般是1/6到1/4屏幕大小，且分配的时候使用heap_caps_malloc函数，第二个参数使用MALLOC_CAP_DMA，确保DMA能直接访问。
2. 通常只会默认加载14号字体，如果要显示其他字体需要在Font usage中勾选对应的字号。
3. 部分显示屏幕比较特殊，执行st7789_flush的时候，起始位置不是从(0,0)，而是(20, 20)，需要加上对应偏移量