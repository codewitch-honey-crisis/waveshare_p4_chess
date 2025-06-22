
#include "soc/soc_caps.h"
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#include "esp_dma_utils.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7703.h"
#include "esp_lcd_touch_gt911.h"
#include <gfx.hpp>
#include <uix.hpp>
#include "chess.h"
#define CB64_IMPLEMENTATION
#include "assets/cb64.hpp"
using namespace gfx;
using namespace uix;

#define LCD_H_RES (720)
#define LCD_V_RES (720)
#define LCD_BIT_PER_PIXEL (16)
#define LCD_PIN_NUM_LCD_RST (27)
#define LCD_PIN_NUM_BK_LIGHT (26) // set to -1 if not used
#define LCD_BK_LIGHT_ON_LEVEL (0)
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL

#if LCD_BIT_PER_PIXEL == 24
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB888)
#elif LCD_BIT_PER_PIXEL == 18
// not supported
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB666)
#elif LCD_BIT_PER_PIXEL == 16
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)
#endif

#define DELAY_TIME_MS (3000)

#define MIPI_DSI_PHY_PWR_LDO_CHAN (3)
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV (2500)

#define LCD_TRANSFER_SIZE (LCD_H_RES*(LCD_V_RES/10)*((LCD_BIT_PER_PIXEL+7)/8))

static uix::display lcd;
static uint8_t *lcd_transfer_buffer=nullptr, *lcd_transfer_buffer2=nullptr;
static esp_ldo_channel_handle_t ldo_mipi_phy;
static esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
static esp_lcd_panel_io_handle_t mipi_dbi_io;
static esp_lcd_panel_handle_t panel_handle;

static esp_lcd_touch_handle_t touch_handle;

static IRAM_ATTR bool lcd_on_flush_complete(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
    lcd.flush_complete();
    return true;
}
static void uix_on_flush(const rect16& bounds, const void* bmp, void* state) {
    esp_lcd_panel_draw_bitmap(panel_handle,bounds.x1,bounds.y1,bounds.x2+1,bounds.y2+1,bmp);
}
static void uix_on_touch(point16* out_locations, size_t* in_out_locations_size, void* state) {
    if(!*in_out_locations_size) {
        return;
    }
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_handle));
    uint16_t x,y,s;
    uint8_t count;
    if(esp_lcd_touch_get_coordinates(touch_handle,&x,&y,&s,&count,1)) {
        out_locations[0]=point16(x,y);
        *in_out_locations_size = 1;
        return;
    }
    *in_out_locations_size = 0;
}
static void lcd_init(void)
{
#if LCD_PIN_NUM_BK_LIGHT >= 0
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL));
#endif

    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
#ifdef MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
#endif

    esp_lcd_dsi_bus_config_t bus_config = ST7703_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));
    
    esp_lcd_dbi_io_config_t dbi_config = ST7703_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));
    esp_lcd_dpi_panel_config_t::extra_dpi_panel_flags dpi_flags = {
        .use_dma2d = true,
    };

    esp_lcd_dpi_panel_config_t dpi_config=// = ST7703_720_720_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    {   
        .virtual_channel = 0,                                                                             
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,     
        .dpi_clock_freq_mhz = 46,                        
        .pixel_format = MIPI_DPI_PX_FORMAT,         
        .num_fbs = 1,                                    
        .video_timing = {                                
            .h_size = 720,                               
            .v_size = 720,                               
            .hsync_pulse_width = 20,                     
            .hsync_back_porch = 80,                     
            .hsync_front_porch = 80,                    
            .vsync_pulse_width = 4,                     
            .vsync_back_porch = 12,                      
            .vsync_front_porch = 30,                     
        },                                               
        .flags= dpi_flags,
    };
    st7703_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
        .flags = {
            .use_mipi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7703(mipi_dbi_io, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    esp_lcd_dpi_panel_event_callbacks_t cbs = {
        .on_color_trans_done = lcd_on_flush_complete,
    };
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(panel_handle, &cbs, NULL));
    lcd_transfer_buffer = (uint8_t*)heap_caps_malloc(LCD_TRANSFER_SIZE,MALLOC_CAP_SPIRAM);
    lcd_transfer_buffer2 = (uint8_t*)heap_caps_malloc(LCD_TRANSFER_SIZE,MALLOC_CAP_SPIRAM);
    if(lcd_transfer_buffer==nullptr||lcd_transfer_buffer2==nullptr) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    lcd.buffer_size(LCD_TRANSFER_SIZE);
    lcd.buffer1(lcd_transfer_buffer);
    lcd.buffer2(lcd_transfer_buffer2);
    lcd.on_flush_callback(uix_on_flush);

}
static void touch_init(void) {

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 7,
        .scl_io_num = 8,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE
    };
    i2c_conf.master.clk_speed = 100*1000;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0));

    /* Initialize touch HW */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_NUM_0, &tp_io_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &touch_handle));
    lcd.on_touch_callback(uix_on_touch);
}

using pixel_t = rgb_pixel<LCD_BIT_PER_PIXEL>;
using color_t = color<pixel_t>;
using uix_color = color<rgba_pixel<32>>;

using screen_t = screen<pixel_t>;

template <typename ControlSurfaceType>
class chess_board : public control<ControlSurfaceType> {
    using base_type = control<ControlSurfaceType>;
    chess_game_t game;
    chess_index_t moves[64];
    chess_size_t moves_size;
    chess_index_t touched;
    spoint16 last_touch;
    
    int move_count;
    void init_board() {
        chess_init(&game);
        moves_size = 0;
        touched = -1;
    }

    int point_to_square(spoint16 point) {
        const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
        const int x = point.x / (extent / 8);
        const int y = point.y / (extent / 8);
        return y * 8 + x;
    }
    void square_coords(int index, srect16* out_rect) {
        const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
        const ssize16 square_size(extent / 8, extent / 8);
        const int x = index % 8;
        const int y = index / 8;
        const spoint16 origin(x * (extent / 8), y * (extent / 8));
        *out_rect = srect16(origin, square_size);
    }
    static const const_bitmap<alpha_pixel<4>>& chess_icon(chess_id_t id) {
        const chess_type_t type = CHESS_TYPE(id);
        switch (type) {
            case CHESS_PAWN:
                return cb64_chess_pawn;
            case CHESS_KNIGHT:
                return cb64_chess_knight;
            case CHESS_BISHOP:
                return cb64_chess_bishop;
            case CHESS_ROOK:
                return cb64_chess_rook;
            case CHESS_QUEEN:
                return cb64_chess_queen;
            case CHESS_KING:
                return cb64_chess_king;
        }
        assert(false);  // invalid piece
        return cb64_chess_pawn;
    }

   public:
    using control_surface_type = ControlSurfaceType;
    using pixel_type = typename ControlSurfaceType::pixel_type;
    using palette_type = typename ControlSurfaceType::palette_type;
    /// @brief Moves a chess_board control
    /// @param rhs The control to move
    chess_board(chess_board&& rhs) {
        do_move_control(rhs);
    }
    /// @brief Moves a chess_board control
    /// @param rhs The control to move
    /// @return this
    chess_board& operator=(chess_board&& rhs) {
        do_move_control(rhs);
        return *this;
    }
    /// @brief Copies a chess_board control
    /// @param rhs The control to copy
    chess_board(const chess_board& rhs) {
        do_copy_control(rhs);
    }
    /// @brief Copies a chess_board control
    /// @param rhs The control to copy
    /// @return this
    chess_board& operator=(const chess_board& rhs) {
        do_copy_control(rhs);
        return *this;
    }
    /// @brief Constructs a chess_board from a given parent with an optional palette
    /// @param parent The parent the control is bound to - usually the screen
    /// @param palette The palette associated with the control. This is usually the screen's palette.
    chess_board(invalidation_tracker& parent, const palette_type* palette = nullptr) : base_type(parent, palette) {
        init_board();
    }
    /// @brief Constructs a chess_board from a given parent with an optional palette
    chess_board() : base_type() {
        init_board();
    }

   protected:
    void do_move_control(chess_board& rhs) {
        do_copy_control(rhs);
    }
    void do_copy_control(chess_board& rhs) {
        memcpy(&game, &rhs.game, sizeof(game));
        if (rhs.moves_size) {
            memcpy(moves, rhs.moves, rhs.moves_size * sizeof(int));
        }
        moves_size = rhs.moves_size;
        move_count = rhs.move_count;
        last_touch = rhs.last_touch;
    }
    void on_paint(control_surface_type& destination, const srect16& clip) override {
        const int16_t extent = destination.dimensions().aspect_ratio() >= 1 ? destination.dimensions().height : destination.dimensions().width;
        const ssize16 square_size(extent / 8, extent / 8);
        bool toggle = false;
        int idx = 0;
        for (int y = 0; y < extent; y += square_size.height) {
            int i = toggle;
            for (int x = 0; x < extent; x += square_size.width) {
                const srect16 square(spoint16(x, y), square_size);
                if (square.intersects(clip)) {
                    const chess_id_t id = chess_index_to_id(&game,idx);
                    pixel_type px_bg = (i & 1) ? color_t::brown : color_t::dark_khaki;
                    pixel_type px_bd = (i & 1) ? color_t::gold : color_t::black;
                    if (id > -1 && CHESS_TYPE(id) == CHESS_KING && chess_status(&game,CHESS_TEAM(id)) == CHESS_CHECK) {
                        px_bd = color_t::red;
                    }
                    if (touched == idx || chess_contains_move(moves, moves_size, idx)) {
                        px_bg = color_t::light_blue;
                        px_bd = color_t::cornflower_blue;
                    }
                    draw::filled_rectangle(destination, square, px_bg);
                    draw::rectangle(destination, square.inflate(-2, -2), px_bd);
                    if (CHESS_NONE != id) {
                        auto ico = chess_icon(id);
                        const srect16 bounds = ((srect16)ico.bounds()).center(square_size.bounds()).offset(x, y);
                        pixel_type px_piece = CHESS_TEAM(id) ? color_t::white : color_t::black;
                        draw::icon(destination, bounds.location(), ico, px_piece);
                    }
                }
                ++i;
                ++idx;
            }
            toggle = !toggle;
        }
    }
    bool on_touch(size_t locations_size, const spoint16* locations) {
        if (touched > -1) {
            if (locations_size) last_touch = locations[0];
            return true;
        }
        if (locations_size) {
            const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
            const srect16 square(spoint16::zero(), ssize16(extent / 8, extent / 8));
            int sq = point_to_square(*locations);
            if (sq > -1) {
                const chess_id_t id = chess_index_to_id(&game,sq);
                if (id > -1) {
                    const chess_team_t team = CHESS_TEAM(id);
                    if (chess_turn(&game) == team) {
                        touched = sq;
                        moves_size = chess_compute_moves(&game,sq,moves);
                        srect16 sq_bnds;
                        square_coords(sq, &sq_bnds);
                        this->invalidate(sq_bnds);
                    }
                }
                if (moves_size > 0) {
                    for (size_t i = 0; i < moves_size; ++i) {
                        srect16 sq_bnds;
                        square_coords(moves[i], &sq_bnds);
                        this->invalidate(sq_bnds);
                    }
                    return true;
                }
            }
        }
        return false;
    }
    void on_release() override {
        if (touched > -1) {
            const chess_id_t id = chess_index_to_id(&game,touched);
            const bool is_king = (CHESS_TYPE(id) == CHESS_KING);
            const chess_team_t team = CHESS_TEAM(id);
            const int16_t extent = this->dimensions().aspect_ratio() >= 1 ? this->dimensions().height : this->dimensions().width;
            const srect16 square(spoint16::zero(), ssize16(extent / 8, extent / 8));
            const int x = touched % 8 * (extent / 8), y = touched / 8 * (extent / 8);
            this->invalidate(square.offset(x, y));
            if (moves_size > 0) {
                for (size_t i = 0; i < moves_size; ++i) {
                    srect16 sq_bnds;
                    square_coords(moves[i], &sq_bnds);
                    this->invalidate(sq_bnds);
                }
                const int release_idx = point_to_square(last_touch);
                if (release_idx != -1) {
                    chess_index_t mv = chess_move(&game,touched,release_idx);
                    if(mv!=-2) {
                        srect16 sq_bnds;
                        square_coords(release_idx, &sq_bnds);
                        this->invalidate(sq_bnds);
                        if(mv>-1 && mv!=release_idx) { // en passant
                            square_coords(mv,&sq_bnds);
                            this->invalidate(sq_bnds);
                        }
                    }
                }
                moves_size = 0;
            }
        }
        touched = -1;
    }
};

screen_t main_screen;

using surface_t = screen_t::control_surface_type;
using chess_board_t = chess_board<surface_t>;

chess_board_t board;

void loop();
void loop_task(void* arg) {
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    while(true) {
        // tickle the watchdog periodically
        if(pdTICKS_TO_MS(xTaskGetTickCount()) >= ms+200) {
            ms = pdTICKS_TO_MS(xTaskGetTickCount());
            vTaskDelay(5);
        }
        loop();
    }
}
extern "C" void app_main() {
    lcd_init();
    touch_init();
    main_screen.dimensions({LCD_H_RES,LCD_V_RES});
    main_screen.background_color(color_t::black);
    board.bounds(srect16(spoint16::zero(),ssize16(64*8,64*8)).center(main_screen.bounds()));
    main_screen.register_control(board);
    
    lcd.active_screen(main_screen);

    TaskHandle_t loop_handle;
    xTaskCreate(loop_task,"loop_task",4096,nullptr,uxTaskPriorityGet(NULL),&loop_handle);
}
void loop() {
    lcd.update();
}