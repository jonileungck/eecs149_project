#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <pthread.h>
#include "httpd.h"

uint8_t data[12];

void *parseRTT(void *args) {
  char str[100];
  //Skip the header lines
  for(int i = 0; i < 25; i++) {
    gets(str);
  }

  while(1) {
    scanf("data: [%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]\n", &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7], &data[8], &data[9], &data[10], &data[11]);
  }
  return NULL;
}

int main( ) {
   pthread_t thread_id;
   pthread_create(&thread_id, NULL, parseRTT, NULL);
   serve_forever("12913");
   return 0;
}

void route()
{
    ROUTE_START()

    ROUTE_GET("/")
    {
        printf("HTTP/1.1 200 OK\r\n\r\n");
        // printf("%s\n", request_header("User-Agent"));
        printf("{'data':[%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u]}", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);
    }

    ROUTE_POST("/")
    {
        printf("HTTP/1.1 200 OK\r\n\r\n");
    }

    ROUTE_END()
}
