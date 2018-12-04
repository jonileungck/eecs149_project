#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

int main( ) {
   char str[100];
   uint8_t data[12];

   //Skip the header lines
   for(int i = 0; i < 25; i++) {
     gets(str);
   }

   while(1) {
     scanf("data: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]\n", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7], &data[8], &data[9], &data[10], &data[11]);
     printf("%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);
   }
   return 0;
}
