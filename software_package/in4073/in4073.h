/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "app_util_platform.h"
#include <math.h>

#define RED		22
#define YELLOW		24
#define GREEN		28
#define BLUE		30
#define INT_PIN		5

bool demo_done;

char pckType;
#define P_Size 8
#define P_Header 0b10100000
#define P_headerCheck 0b00001010
char type_pck;
struct packet{
	uint8_t head;
	uint8_t pck_Type;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
	int8_t lift;
	uint16_t crc;
} pc_drone;

struct dronepck{
	uint8_t head;
	uint8_t pckType;
	uint8_t dt1_1;
	uint8_t dt1_2;
	uint8_t dt2_1;
	uint8_t dt2_2;
	uint8_t dt3_1;
	uint8_t dt3_2;
	uint8_t dt4_1;
	uint8_t dt4_2;
} drone_pc;

uint8_t brokenPck[P_Size];

uint8_t mode;
int panic;
int batFlag;
uint8_t timestamp;

struct mode_packet{
	char head_p;
	uint8_t mode_p;
	char ender_p;
}mode_packet_change;
bool mode_change_acknowledged;
uint8_t prevAckMode;

// Control
int16_t motor[4],ae[4];
int32_t kp, kp1, kp2;
int32_t kp_yaw, kp1_roll, kp2_roll, kp1_pitch, kp2_pitch;
int32_t pitch_error, yaw_error, roll_error, lift_error;
int32_t	lift, roll, pitch, yaw;
int32_t err_pitch, err_roll, err_lift, err_yaw;
int32_t cali_p, cali_q, cali_r;
int32_t cali_phi,cali_theta,cali_pressure;


void calculateMotorRPM(void);
void run_filters_and_control();
void update_motors(void);


int p_Flag;
int rawFlag;
int lost_connectFlag;
uint8_t prevAcknowledgeMode;
void (*mode_function)(void);


void raw_ctr(void);
void height_ctr(void);
void roll_cal(void);
void yaw_cal(void);


//filter
int16_t p_est;
int16_t q_est;
int16_t theta_est;
int16_t phi_est;

int16_t r_butter;

void filter_function(void);
//mode
uint8_t pre_mode;
//void switch_mode(void);
void panic_mode(void);
void safe_mode(void);
void manual_mode(void);
void calib_mode(void);
void switch_mode(int);
void yaw_mode(void);;
void full_mode(void);
void raw_mode(void);
void height_mode(void);
void calibration(void);
void manualModePacket();
#define BUFFER_SIZE 200





// Timers
#define TIMER_PERIOD	50 //50ms=20Hz (MAX 23bit, 4.6h)
void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);

// GPIO
void gpio_init(void);

// Queue
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint16_t first,last;
  	uint16_t count; 
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);
void flushQueue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
uint32_t last_correct_checksum_time;
void uart_init(void);
void uart_put(uint8_t);
int uart_put_packet(int number);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
void get_raw_sensor_data(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
volatile bool radio_active;
void ble_init(void);
void ble_send(void);

//dront_to_pc file functions - protocol function
uint16_t compute_crc(const uint8_t *pck_data, uint32_t size, const uint16_t *pck_crc);
void send_mode_change();
void set_acknowledge_flag(bool ack_flag);
void init_send_mode_change();
void send_packet(char type);
void packet_on_queue();
void set_pckType(char temp);
void mode_change_packet();
void motor_packet();
void kp_packet();
void set_Header();
bool check_crc();
bool nxt_packet();
uint8_t mode_set();
void restore_packet();
void pckType_check();
bool brokenPckt_check();
void header_brokenpckt();
void restore_brokenPckt();
bool headerCheck(uint8_t h);
uint8_t read_packet();
void send_panicPckt();

uint16_t compute_crc_1(const uint8_t *pck_data, uint32_t size, const uint16_t *pck_crc);

#endif // IN4073_H__
