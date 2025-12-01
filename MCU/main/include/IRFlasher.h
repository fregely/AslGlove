#ifndef IRFLASHER_H
#define IRFLASHER_H

#include <stdint.h>

void irflasher_start(void);               // Called when Python writes START (1)
void irflasher_next(void);                // Called when Python writes NEXT (2)
void irflasher_notify_ready(uint8_t led); // Called by LED task to notify Python
void irflasher_task(void *arg);
void irflasher_reset(void);              // Called on BLE disconnect
#endif // IRFLASHER_H
