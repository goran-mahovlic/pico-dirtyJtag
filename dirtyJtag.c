#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pio_jtag.h"
#include "cdc_uart.h"
#include "led.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "bsp/board.h"
#include "tusb.h"
#include "cmd.h"
#include "get_serial.h"
#include "dirtyJtagConfig.h"
#include "pico/async_context_threadsafe_background.h"

#define MULTICORE
#define ULX4M

#ifdef ULX4M
#include "hardware/i2c.h"
#include "tca9534.h"
expander_t expander;

#define PWR_BRD_I2C_INST                i2c0
#define PWR_BRD_I2C_SDA                 4
#define PWR_BRD_I2C_SCL                 5
#define PWR_BRD_I2C_BAUD                (10 * 1000)
#define PWR_BRD_I2C_TIMEOUT_US          20000
#define PWR_BRD_EXPANDER_ADDR           0b0100000
#define PWR_BRD_EXPANDER_PIN_DIRS       0b00001111

#define BUFFER_LENGTH 200

bool pwr_brd_charger_power(uint8_t pin, bool on);

const uint ADCMUX1_PIN = 27;
const uint ADCMUX2_PIN = 28;

uint32_t uart_counter = 0;
uint8_t uart_print_counter = 0;
uint8_t last_gpio_state = 0;
bool last_expander_state = 0;
bool last_expander_i2c_state = 0;

bool uart_ready = 0;
bool uart_send = 0;
uint8_t uart_tx_buf[BUFFER_LENGTH];


void send_uart(char *buf){
    sprintf (uart_tx_buf, buf);
}

void runFUN1(){
    printf("FUN1");
}

void set_power(uint8_t module, uint8_t value){
    pwr_brd_charger_power(module+3,value);
    printf("SETPWR;%i;%i",module,value);
}
uint8_t get_power(uint8_t module){
    uint8_t output_state = expander.last_output_state;
    return output_state;
}
void set_adc_mux(uint8_t module){
    if(module == 1){
        gpio_put(ADCMUX1_PIN, 0);gpio_put(ADCMUX2_PIN, 0);
    }
    else if(module == 2){
        gpio_put(ADCMUX1_PIN, 0);gpio_put(ADCMUX2_PIN, 1);
    }
    else if(module == 3){
        gpio_put(ADCMUX1_PIN, 1);gpio_put(ADCMUX2_PIN, 0);
    }
    else if(module == 4){
        gpio_put(ADCMUX1_PIN, 1);gpio_put(ADCMUX2_PIN, 1);
    }
    sleep_ms(500);
}
uint8_t get_status(uint8_t module){
 printf("NOT IMPLEMENTED");
}

void parse_serial(){

    uint8_t command[8];
    uint8_t module[2];
    uint8_t value[2];

    memcpy(command, &cmd_buff[0], 6);
    command[6] = '\0';
    memcpy(module, &cmd_buff[11], 1);
    module[1] = '\0';
    memcpy(value, &cmd_buff[13], 1);
    value[1] = '\0';
    module[0] = module[0] - 48;
    value[0] = value[0] - 48;

    if(module[0] == 1 || module[0] == 2 || module[0] == 3 || module[0] == 4){
        if (strcmp(command, "SETPWR") == 0) {
                if (value[0] == 0 || value[0] == 1){
                    set_power(module[0], value[0]);
                }
                else{
                    printf("Value ERROR!\n\r"); 
                }
        } 
        else if (strcmp(command, "GETPWR") == 0){
                bool power = get_power(module[0]) >> 3+module[0];
                printf("POWER;MOD_%i;%i\n\r",module[0], power);
        }
        else if (strcmp(command, "GETADC") == 0){
            set_adc_mux(module[0]);
            // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
            const float conversion_factor = 3.3f / (1 << 12);
            uint16_t result = adc_read();
            printf("GETADC;module[0];%f\n\r", result * conversion_factor);
        }
        else if (strcmp(command, "STATUS") == 0){
                uint8_t status = get_status(module[0]);
        }
        else{
            printf("Command ERROR!\n\r");
        }
        }
        else{
            printf("No module %i\n\r", module[0]);
        }     
}

void print_task(){

    if(uart_rx_available){
        uart_rx_available = false;
        //printf("Serial available!\n");
        parse_serial();
        //printf(cmd_buff);
    }
/*

    uart_counter++;
    if (uart_counter>=1000000){
        uart_counter = 0;
        uart_print_counter++;
    }
    if(uart_print_counter>=1){
        uart_print_counter = 0;        
        uart_send = 1;
    }
    if(uart_send){
        printf(uart_tx_buf);
        printf("\n");
        // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
        const float conversion_factor = 3.3f / (1 << 12);
        uint16_t result = adc_read();
        printf("Raw value: 0x%03x, voltage: %f V\n", result, result * conversion_factor);
        uart_send = 0;
    }
    */
}

bool pwr_brd_raw_gpio_read(uint8_t* val) {
    return expander_read_inputs(&expander, val);
}

bool ensure_expander_output_dirs(void) {
    static bool is_setup = false;
    if (!is_setup) {
        if (!expander_set_pin_directions(&expander, PWR_BRD_EXPANDER_PIN_DIRS)) {
            return false;
        }
        is_setup = true;
    }
    return true;
}

bool pwr_brd_set_gpio_outs(uint8_t outs) {
    ensure_expander_output_dirs();
    return expander_write_outputs(&expander, outs);
}

bool pwr_brd_charger_power(uint8_t pin, bool on) {
    uint8_t output_state = expander.last_output_state;
    last_gpio_state = output_state;
    if (on) {
        output_state |= (1 << pin);
    } else {
        output_state &= ~(1 << pin);
    }

    return pwr_brd_set_gpio_outs(output_state);
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

#endif

void init_pins()
{
    bi_decl(bi_4pins_with_names(PIN_TCK, "TCK", PIN_TDI, "TDI", PIN_TDO, "TDO", PIN_TMS, "TMS"));
    #if !( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )
    bi_decl(bi_2pins_with_names(PIN_RST, "RST", PIN_TRST, "TRST"));
    #endif
}

pio_jtag_inst_t jtag = {
            .pio = pio0,
            .sm = 0
};

void djtag_init()
{
    init_pins();
    #if !( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )
    init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, PIN_RST, PIN_TRST);
    #else
    init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, 255, 255);
    #endif
}
typedef uint8_t cmd_buffer[64];
static uint wr_buffer_number = 0;
static uint rd_buffer_number = 0; 
typedef struct buffer_info
{
    volatile uint8_t count;
    volatile uint8_t busy;
    cmd_buffer buffer;
} buffer_info;

#define n_buffers (4)

buffer_info buffer_infos[n_buffers];

static cmd_buffer tx_buf;

void jtag_main_task()
{
#ifdef MULTICORE
    if (multicore_fifo_rvalid())
    {
        //some command processing has been done
        uint rx_num = multicore_fifo_pop_blocking();
        buffer_info* bi = &buffer_infos[rx_num];
        bi->busy = false;

    }
#endif
    if ((buffer_infos[wr_buffer_number].busy == false)) 
    {
        //If tud_task() is called and tud_vendor_read isn't called immediately (i.e before calling tud_task again)
        //after there is data available, there is a risk that data from 2 BULK OUT transaction will be (partially) combined into one
        //The DJTAG protocol does not tolerate this. 
        tud_task();// tinyusb device task
        if (tud_vendor_available())
        {
            led_rx( 1 );
            printf(tx_buf);
            uint bnum = wr_buffer_number;
            uint count = tud_vendor_read(buffer_infos[wr_buffer_number].buffer, 64);
            if (count != 0)
            {
                buffer_infos[bnum].count = count;
                buffer_infos[bnum].busy = true;
                wr_buffer_number = wr_buffer_number + 1; //switch buffer
                if (wr_buffer_number == n_buffers)
                {
                    wr_buffer_number = 0; 
                }
#ifdef MULTICORE
                multicore_fifo_push_blocking(bnum);
#endif
            }
            led_rx( 0 );
        } else {
#if ( USB_CDC_UART_BRIDGE || USB_CDC_UART_BRIDGE_FAKE)           
            cdc_uart_task();
#endif
        }
    }
}

void jtag_task()
{
#ifndef MULTICORE
    jtag_main_task();
#endif
}

#ifdef MULTICORE
void core1_entry() {

    djtag_init();
    while (1)
    {
        uint rx_num = multicore_fifo_pop_blocking();
        buffer_info* bi = &buffer_infos[rx_num];
        assert (bi->busy);
        cmd_handle(&jtag, bi->buffer, bi->count, tx_buf);
        multicore_fifo_push_blocking(rx_num);
    }
 
}
#endif

void fetch_command()
{
#ifndef MULTICORE
    if (buffer_infos[rd_buffer_number].busy)
    {
        cmd_handle(&jtag, buffer_infos[rd_buffer_number].buffer, buffer_infos[rd_buffer_number].count, tx_buf);
        buffer_infos[rd_buffer_number].busy = false;
        rd_buffer_number++; //switch buffer
        if (rd_buffer_number == n_buffers)
        {
            rd_buffer_number = 0; 
        }
    }
#endif
}

//this is to work around the fact that tinyUSB does not handle setup request automatically
//Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;
    return false;
}

int main()
{
    stdio_init_all();
    sleep_ms(2000);
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    // Initialize ADCMUX select pins
    //
    gpio_init(ADCMUX1_PIN);
    gpio_set_dir(ADCMUX1_PIN, GPIO_OUT);
    gpio_init(ADCMUX2_PIN);
    gpio_set_dir(ADCMUX2_PIN, GPIO_OUT);
    gpio_put(ADCMUX1_PIN, 0);
    gpio_put(ADCMUX2_PIN, 0);

    board_init();
    usb_serial_init();
    tusb_init();

    led_init( LED_INVERTED, PIN_LED_TX, PIN_LED_RX, PIN_LED_ERROR );
#if ( USB_CDC_UART_BRIDGE )
    cdc_uart_init( PIN_UART0, PIN_UART0_RX, PIN_UART0_TX );
    #if (PIN_UART_INTF_COUNT == 2)
        cdc_uart_init( PIN_UART1, PIN_UART1_RX, PIN_UART1_TX );
    #endif
#endif

#ifdef MULTICORE
    multicore_launch_core1(core1_entry);
#else 
    djtag_init();
#endif
#ifdef ULX4M
 
        i2c_init(PWR_BRD_I2C_INST, PWR_BRD_I2C_BAUD);
        //gpio_pull_up(PWR_BRD_I2C_SDA);
        //gpio_pull_up(PWR_BRD_I2C_SCL);
        gpio_set_function(PWR_BRD_I2C_SDA, GPIO_FUNC_I2C);
        gpio_set_function(PWR_BRD_I2C_SCL, GPIO_FUNC_I2C);
        // Make the I2C pins available to picotool
        //bi_decl(bi_2pins_with_func(PWR_BRD_I2C_SDA, PWR_BRD_I2C_SCL, GPIO_FUNC_I2C));
        expander_init(&expander, PWR_BRD_I2C_INST, PWR_BRD_EXPANDER_ADDR, PWR_BRD_I2C_TIMEOUT_US);
        pwr_brd_charger_power(4,1);
        pwr_brd_charger_power(5,1);
        pwr_brd_charger_power(6,1);
        pwr_brd_charger_power(7,1);
        //pwr_brd_charger_power(8,1);                            
        //pwr_brd_charger_power(module+3,value);
        //pwr_brd_charger_power(module+3,value);
        send_uart("Init done!");
#endif

    while (1) {
        jtag_main_task();
        fetch_command(); //for unicore implementation
        print_task();

    }
}
