#ifndef __LAB07_MOTOR_H_INCLUDED_
#define __LAB07_MOTOR_H_INCLUDED_

typedef enum {M_SLOW, M_MEDIUM, M_FAST} motor_speed_t;
typedef enum {M_BUSY, M_IDLE} motor_status_t;

/*
** There are much better templates in C++ but
** these will work here just fine.
*/
#define min(x, y) (((x) > (y)) ? (y) : (x))
#define max(x, y) (((x) < (y)) ? (x) : (y))
#define abs(x)    ((x) < 0) ? ((x) * -1) : (x)

// const uint8_t PHA_PIN = 5;
// const uint8_t PHB_PIN = 4;
// const uint8_t CT1_PIN = 6;
// const uint8_t CT2_PIN = 7;

void motor_init(double Kp, double Ki, double Kd, bool enableISR);
void motor_speed_limit(motor_speed_t speed);
void motor_move(uint32_t pos_in_tics);
int32_t motor_get_position(void);
motor_status_t motor_status(void);
void set_motor_status(motor_status_t status);
void phase_change_irq(unsigned int gpio, long unsigned int event);

#endif
