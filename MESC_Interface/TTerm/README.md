# TTerm
TTern is a lightweight and portable CLI for embedded applications. 

Key features are:
- full tab-autocomplete
- optional dynamic parameter autocomplete
- command history
- support for multiple terminal endpoints at once
- support for custom command lists for different endpoints
- supports cwd in combination with FatFs
- support for "programms" overriding the terminal
- task safe

Adding commands is also easy:

```
uint8_t CMD_cls(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    uint8_t currArg = 0;
    uint8_t returnCode = TERM_CMD_EXIT_SUCCESS;
    for(;currArg<argCount; currArg++){
        if(strcmp(args[currArg], "-?") == 0){
            ttprintf("clears the screen\r\n");
            return TERM_CMD_EXIT_SUCCESS;
        }
    }
    
    TERM_sendVT100Code(handle, _VT100_RESET, 0); TERM_sendVT100Code(handle, _VT100_CURSOR_POS1, 0);
    TERM_printBootMessage(handle);
    
    return TERM_CMD_EXIT_SUCCESS;
}

void init(){
    TERM_addCommand(CMD_cls, "cls", "Clears the screen", 0, &TERM_defaultList);
}
```

## documentation is still in the making though...
