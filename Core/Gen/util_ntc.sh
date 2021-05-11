
ENVCC=${CC:-gcc}
echo "INFO: Using ${ENVCC}"
${ENVCC} -Wall -Wextra -pedantic -std=c11 -g -ggdb3 -O0 ntc.c util_ntc.c -lm -o util_ntc
