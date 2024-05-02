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

void motor_init(double Kp, double Ki, double Kd);
void motor_speed_limit(motor_speed_t speed);
void motor_move(uint32_t pos_in_tics);
int32_t motor_get_position(void);
motor_status_t motor_status(void);
void set_motor_status(motor_status_t status);

#endif
