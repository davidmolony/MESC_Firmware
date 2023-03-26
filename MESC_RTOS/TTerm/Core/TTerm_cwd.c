/*
 * TTerm
 *
 * Copyright (c) 2021 Thorben Zethoff, Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "TTerm/Core/include/TTerm.h"
#if TERM_SUPPORT_CWD == 1

#include "TTerm/Core/include/TTerm_cwd.h"
#include "FreeRTOS.h"
#include "ff.h"
#include "ffconf.h"
#include <string.h>

#define BUFFER_SIZE 255

uint8_t FS_dirUp(char * path);
char * FS_newCWD(char * oldPath, char * newPath);

char * FS_newCWD(char * oldPath, char * newPath){
    uint8_t count = 0;

    char * cp = newPath;
    while(*(cp) != 0){
        if(*cp == '/') count ++;
        cp++;
    }

    count ++;
    char ** pathParts = pvPortMalloc(sizeof(char *) * count);

    cp = newPath;
    uint8_t currPart = 0;
    pathParts[0] = newPath;
    while(*(cp) != 0){
        if(*cp == '/'){
            pathParts[++currPart] = cp + 1;
            *cp = 0;
        }
        cp++;
    }

    currPart = 0;
    char * temp = pvPortMalloc(_MAX_LFN+1);
    unsigned slashSkip = 0;

    if(*pathParts[currPart] != 0){  //the first position is a valid name -> append to current path
        #ifdef DEBUG_CWD
        printf("copied old path (appending)\r\n");
        #endif
        strncpy(temp, oldPath, _MAX_LFN);
        uint16_t len = strlen(temp);
        if(temp[len - 1] != '/'){
            temp[len] = '/';
            temp[len + 1] = 0;
        }
        slashSkip = 1;
    }else{
        strcpy(temp, "/");
        slashSkip = 1;
        currPart++;
    }

    for(; currPart < count; currPart++){
        if(*pathParts[currPart] == 0){
            #ifdef DEBUG_CWD
        	printf("skipped empty part\r\n", currPart, pathParts[currPart]);
            #endif
        }else if(strcmp(pathParts[currPart], "..") == 0){
            FS_dirUp(temp);
            slashSkip = 1;
            #ifdef DEBUG_CWD
            printf("Part %d: \"%s\" (go back one)\r\n", currPart, pathParts[currPart]);
            #endif
        }else if(strcmp(pathParts[currPart], ".") == 0){
            #ifdef DEBUG_CWD
        	printf("Part %d: \"%s\" (skipped)\r\n", currPart, pathParts[currPart]);
            #endif
        }else{
            sprintf(temp, "%s%s%s", temp, slashSkip ? "" : "/", pathParts[currPart]);
            slashSkip = 0;
            #ifdef DEBUG_CWD
            printf("Part %d: \"%s\" (appended)\r\n", currPart, pathParts[currPart]);
            #endif
        }
    }

    uint16_t len = strlen(temp);

    char * ret = pvPortMalloc(len + 1);
    strcpy(ret, temp);
    #ifdef DEBUG_CWD
    printf("newPath = \"%s\"\r\n", ret);
    #endif

    vPortFree(temp);
    vPortFree(pathParts);

    return ret;
}

uint8_t FS_dirUp(char * path){
    uint16_t currPos = strlen(path) - 1;

    if(currPos == 0) return 0;

    if(path[currPos] == '/'){
        path[currPos--] = 0;
    }

    while(path[currPos] != '/'){
        currPos --;
        if(currPos == 0) break;
    }

    path[currPos+1] = 0;

    return 1;
}

uint8_t CMD_cat(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    
    if(argCount==0) return TERM_CMD_EXIT_SUCCESS;
    
    uint8_t* buffer = pvPortMalloc(BUFFER_SIZE);

    if(buffer==NULL) {
        ttprintf("Cannot allocate memory\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(argCount==1){ 
        char * filePath = FS_newCWD(handle->cwdPath, args[0]);
        FIL fp;
        FRESULT res = f_open(&fp, filePath,FA_READ);
        if(res != FR_OK){
            ttprintf("Error file open\r\n");
            vPortFree(filePath);
            vPortFree(buffer);
            return TERM_CMD_EXIT_SUCCESS;
        }

        unsigned int br=0;
        do{
        	f_read(&fp, buffer, BUFFER_SIZE, &br);
        	ttprintf(NULL, buffer, br);
        }while(br);

        f_close(&fp);
        vPortFree(buffer);
        vPortFree(filePath);
        
    } else if(argCount>2){
        uint8_t n_files = argCount-2;
        
        FIL fp;

        BYTE mode;
        if(strcmp(args[argCount-2], ">")==0){
            mode = FA_WRITE | FA_CREATE_ALWAYS;
        }else if(strcmp(args[argCount-2], ">>")==0){
            mode = FA_WRITE | FA_OPEN_APPEND;
        }else{
            ttprintf("Syntax error\r\n");
            vPortFree(buffer);
        }
        FIL out;
        char * filePath = FS_newCWD(handle->cwdPath, args[argCount-1]);
        FRESULT res = f_open(&out,filePath,mode);
        if(res != FR_OK){
            ttprintf("Error creating file\r\n");
            vPortFree(buffer); 
            vPortFree(filePath);
        }
        uint32_t bytes_read;
        uint32_t bytes_write;
        uint32_t bytes_sum=0;
        FRESULT res_read;
        FRESULT res_write;
        uint8_t i;
        for(i=0;i<n_files;i++){
            char * readFilePath = FS_newCWD(handle->cwdPath, args[i]);
            FRESULT res = f_open(&fp,readFilePath,FA_READ);
            if(res != FR_OK){
                ttprintf("Error file open\r\n");
                vPortFree(buffer);
                vPortFree(readFilePath);
                return TERM_CMD_EXIT_SUCCESS;
            }
        
            do{
                res_read = f_read(&fp, buffer, BUFFER_SIZE, &bytes_read);
                res_write = f_write(&out, buffer, bytes_read, &bytes_write);
                bytes_sum += bytes_write;
            }while(res_read == FR_OK && res_write == FR_OK && bytes_read);
            vPortFree(readFilePath);
            f_close(&fp);
        }
        vPortFree(filePath);
        vPortFree(buffer);
        ttprintf("%u bytes written\r\n", bytes_sum);
        f_close(&out);
    }
    
    return TERM_CMD_EXIT_SUCCESS;
}

void conv_esc(char * ptr){
    char code;
    while(*ptr){
        code = 0;
        if(*ptr == '\\'){
            switch(*(ptr+1)){
                case 'r':
                    code = '\r';
                    break;
                case 'n':
                    code = '\n';
                    break;  
                case 't':
                    code = '\t';
                    break;
                case '\\':
                    code = '\\';
                    break;
            }
            if(code){
                *ptr = code;
                char * cpy_source = ptr + 2;
                char * cpy_dest = ptr + 1;     
                while(*cpy_source){
                    *cpy_dest = *cpy_source;
                    *cpy_dest++;
                    *cpy_source++;
                }
                if(*cpy_source == 0) *cpy_dest = 0;
            }
        }
        ptr++;
    }
}

uint8_t CMD_echo(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    
    if(argCount==0) return TERM_CMD_EXIT_SUCCESS;
    
    if(argCount==1){
        conv_esc(args[0]);
        ttprintf("%s", args[0]);
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    if(argCount>2){

        BYTE mode;
        if(strcmp(args[argCount-2], ">")==0){
            mode = FA_WRITE | FA_CREATE_ALWAYS;
        }else if(strcmp(args[argCount-2], ">>")==0){
            mode = FA_WRITE | FA_OPEN_APPEND;
        }else{
            ttprintf("Syntax error\r\n");
            return TERM_CMD_EXIT_SUCCESS;
        }
        FIL out;
        char * filePath = FS_newCWD(handle->cwdPath, args[argCount-1]);

        FRESULT res = f_open(&out,filePath,mode);
        if(res != FR_OK){
            vPortFree(filePath);
            ttprintf("Error creating file\r\n");
            return TERM_CMD_EXIT_SUCCESS;
        }
        //vTaskDelay(10);
        if(argCount==3){  
            conv_esc(args[0]);
            ttprintf(args[0]);
            f_puts(args[0], &out);
        }
        
        f_close(&out);
        vPortFree(filePath);
    }

    return TERM_CMD_EXIT_SUCCESS;
}

int EndsWith(const char *str, const char *suffix)
{
    if (!str || !suffix)
        return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix >  lenstr)
        return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

//generate a unix style directory list item, but without the time stuff
//for reference: https://www.ibm.com/support/knowledgecenter/ssw_ibm_i_74/rzaiq/rzaiqrzaiqfdrtu.htm
void CWD_generateDirlistEntry(TERMINAL_HANDLE * handle, FILINFO * fno){
    
    //is the entry a file or a directory?
    char type = '-';
    if(fno->fattrib & AM_DIR) type = 'd';
    
    //yeah the permissions are constant for now...
    char permissions[] = "rw-rw-rw-";
    
    if(type == 'd'){
        ttprintf("%c%s          %s%s%s\r\n", type, permissions, TERM_getVT100Code(_VT100_FOREGROUND_COLOR,_VT100_GREEN) ,fno->fname, TERM_getVT100Code(_VT100_FOREGROUND_COLOR,_VT100_WHITE));
    }else{
        ttprintf("%c%s %8d %s\r\n", type, permissions, fno->fsize, fno->fname, TERM_getVT100Code(_VT100_FOREGROUND_COLOR,_VT100_RED));
    }
}

uint8_t CMD_ls(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    DIR dir;
    FRESULT res = f_opendir(&dir, handle->cwdPath);
    FILINFO fno;
    
    if(res == FR_OK){
        while(1){
            //read the next entry
            res = f_readdir(&dir, &fno);          
            
            //check if it is valid, and not hidden
            if (res != FR_OK || fno.fname[0] == 0) break;  
            if(fno.fattrib & (AM_HID | AM_SYS)) continue;
            
            //Print String
            CWD_generateDirlistEntry(handle, &fno);
            
        }
        //clean up and send the last buffer
        f_closedir(&dir);
    }

    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;


    /* Get volume information and free clusters of drive 1 */
    res = f_getfree("0:", &fre_clust, &fs);
    if (res) return TERM_CMD_EXIT_SUCCESS;

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    /* Print the free space (assuming 512 bytes/sector) */
    ttprintf("\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", tot_sect / 2, fre_sect / 2);

    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_cd(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount == 0){
        ttprintf("\r\n");
        return TERM_CMD_EXIT_ERROR;
    }
    
    if(strcmp(args[0], "..")==0){
        FS_dirUp(handle->cwdPath);  
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    //generate the new CWD path. This can include this like ../ and ./
    char * oldPath = handle->cwdPath;
    char * newCWD = FS_newCWD(oldPath, args[0]);
    
    //check if the new path is valid
    DIR temp;
    FRESULT res = f_opendir(&temp, newCWD);                
    if(res == FR_OK){
        f_closedir(&temp);
        
        //update the cwd Path in the FTP_CLIENT_HANDLE
        vPortFree(handle->cwdPath);
        handle->cwdPath = newCWD;

    }else{
        //free the new string, if the path is invalid
        vPortFree(newCWD);
    }
    
    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_mkdir(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount == 0){
        return TERM_CMD_EXIT_ERROR;
    }
    
    FRESULT res;
    DIR dir;

    char * dirPath = FS_newCWD(handle->cwdPath, args[0]);
    
    //check if the directory exists
    res = f_opendir(&dir, dirPath);
    if(res == FR_OK){
        f_closedir(&dir);
        vPortFree(dirPath);
        ttprintf("Directory already exists\r\n");
        return TERM_CMD_EXIT_SUCCESS;
    }
    
    //create it
    res = f_mkdir(dirPath);
    if(res != FR_OK){
        ttprintf("Didn't work (%d) :(\r\n", res);
    }
    
    vPortFree(dirPath);
    
    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_rm(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    if(argCount == 0){
        return TERM_CMD_EXIT_ERROR;
    }

    FRESULT res;

    char * dirPath = FS_newCWD(handle->cwdPath, args[0]);


    //delete file
    res = f_unlink(dirPath);
    if(res != FR_OK){
        ttprintf("File or directory not found\r\n");
    }else{
    	ttprintf("%s deleted.", dirPath);
    }




    vPortFree(dirPath);

    return TERM_CMD_EXIT_SUCCESS;
}

#endif
