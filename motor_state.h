
typedef enum
{
    
} motor_state_e;

typedef uint16_t motor_phase_t; // TODO should this be float?
// TODO quantisation?

typedef enum
{
    MOTOR_SENSOR_MODE_OPENLOOP,
    MOTOR_SENSOR_MODE_HALL,
    MOTOR_SENSOR_MODE_NONE,
} motor_sensor_mode_e;

typedef enum
{
    MOTOR_DIRECTION_CLOCKWISE,
    MOTOR_DIRECTION_COUNTERCLOCKWISE
} motor_direction_e;

typedef struct
{
    motor_phase_t Rphase;
    motor_phase_t Lphase;
    uint16_t      RawCurrLim;
    uint16_t      RawVoltLim;
} motor_s;

void motor_init( motor_s * motor );

