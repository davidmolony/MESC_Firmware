
#define FOC_SECTORS_PER_REVOLUTION (6)
#define FOC_CHANNELS               (3)
#define FOC_unknown                (2)

typedef uint16_t foc_angle_t;
typedef int32_t  foc_field_angle_t;

typedef struct
{
    uint16_t    IMid[FOC_CHANNELS]; // TODO type?
    foc_angle_t ElecAngle;
    uint16_t    Sector; // [0,FOC_SECTORS_PER_REVOLUTION)
    foc_angle_t AnglePerSector; // ((USHRT_MAX + 1) / FOC_SECTORS_PER_REVOLUTION)
    foc_angle_t RotorAngle;
    foc_angle_t AngleStep;
    foc_angle_t PWM[FOC_CHANNELS];

    int32_t           Iab[FOC_unknown];
    foc_field_angle_t FieldAngle;
    int32_t           Idq[FOC_unknown];
} foc_s;

void foc_init( foc_s * foc );

#define FOC_NUM_ADC (3)

typedef struct
{
    uint16_t RawADC[FOC_NUM_ADC][FOC_CHANNELS]; // TODO type?
    uint16_t ADCOffset[FOC_NUM_ADC];
    struct
    {
        uint16_t Delta;
        uint16_t Length;
    }        RCPWMin;
} foc_measurement_t;

