
ENVCC=${CC:-gcc}
echo "INFO: Using ${ENVCC}"
${ENVCC} -Wall -Wextra -pedantic -std=c11 -g -ggdb3 -O0 -I../Inc/ ../Src/MESCbat.c ../Src/MESCcli.c ../Src/MESCfnv.c ../Src/MESCprofile.c ../Src/MESCspeed.c ../Src/MESCtemp.c bist*.c -lm -o bist

