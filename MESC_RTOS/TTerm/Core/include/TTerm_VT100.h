

enum vt100{
    _VT100_CURSOR_POS1,
    _VT100_CURSOR_END,
    _VT100_FOREGROUND_COLOR,
    _VT100_BACKGROUND_COLOR,
    _VT100_RESET_ATTRIB,
    _VT100_BRIGHT,
    _VT100_DIM,
    _VT100_UNDERSCORE,
    _VT100_BLINK,
    _VT100_REVERSE,
    _VT100_HIDDEN,
    _VT100_ERASE_SCREEN,
    _VT100_ERASE_LINE,
    _VT100_FONT_G0,
    _VT100_FONT_G1,
    _VT100_WRAP_ON,
    _VT100_WRAP_OFF,
    _VT100_ERASE_LINE_END,
    _VT100_CURSOR_BACK_BY,
    _VT100_CURSOR_FORWARD_BY,
    _VT100_CURSOR_DOWN_BY,
    _VT100_CURSOR_UP_BY,
    _VT100_CURSOR_SAVE_POSITION,
    _VT100_CURSOR_RESTORE_POSITION,
    _VT100_CURSOR_ENABLE,
    _VT100_CURSOR_DISABLE,
    _VT100_CLS
};

//VT100 cmds given to us by the terminal software (they need to be > 8 bits so the handler can tell them apart from normal characters)
#define _VT100_RESET                0x1000
#define _VT100_KEY_END              0x1001
#define _VT100_KEY_POS1             0x1002
#define _VT100_CURSOR_FORWARD       0x1003
#define _VT100_CURSOR_BACK          0x1004
#define _VT100_CURSOR_UP            0x1005
#define _VT100_CURSOR_DOWN          0x1006
#define _VT100_BACKWARDS_TAB        0x1007
#define _VT100_KEY_DEL              0x1008
#define _VT100_KEY_INS              0x1009
#define _VT100_KEY_PAGE_UP          0x100a
#define _VT100_KEY_PAGE_DOWN        0x100b
#define _VT100_INVALID              0xffff


enum color{
    _VT100_BLACK,
    _VT100_RED,
    _VT100_GREEN,
    _VT100_YELLOW,
    _VT100_BLUE,
    _VT100_MAGENTA,
    _VT100_CYAN,
    _VT100_WHITE
};

#define _VT100_POS_IGNORE 0xffff