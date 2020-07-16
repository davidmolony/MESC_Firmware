
#define HW_SETUP_RSHUNT (1000)
//:
//#define HW_SETUP_IGAIN ((HW_SETUP_RSHUNT*...)/(...))
// _OR_
typedef struct
{
    uint16_t Rshunt;
    uint16_t RVBT;
    uint16_t RVBB;
    uint16_t RIphPU;
    uint16_t RIphSR;
    uint16_t OpGain;
    uint16_t Igain;
} hw_setup_s;

const hw_setup_s g_hw_setup;
// _OR_
// void hw_setup_init( hw_setp_s * hw_setup );
//
