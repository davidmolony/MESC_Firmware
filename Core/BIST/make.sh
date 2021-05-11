
ENVCC=${CC:-gcc}
echo "INFO: Using ${ENVCC}"
${ENVCC} -Wall -Wextra -pedantic -std=c11 -g -ggdb3 -O0 -I../Inc/ -I../Gen ../Src/MESCbat.c ../Src/MESCcli.c ../Src/MESCfnv.c ../Src/MESCprofile.c ../Src/MESCspeed.c ../Src/MESCtemp.c ../Gen/ntc.c bist*.c -lm -o bist
