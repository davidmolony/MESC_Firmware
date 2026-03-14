//not-so example config...
//i guess it might be an example but there is hardly anything to configure yet lol

#define TERM_ENABLE_STARTUP_TEXT
#define TERM_CUSTOM_STARTUP_TEXT

#define TERM_SUPPORT_CWD 0
#ifdef STM32L432xx
	#define ALLIGNED_DOUBLE_WORD
	#define TERM_DEVICE_NAME "MESC"
#endif
#define TERM_SUPPORT_APPS 1
#define TERM_DEBUG_UART 0
#define TERM_SUPPORT_VARIABLES 1
