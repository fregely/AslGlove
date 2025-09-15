#ifndef NIMBLE_GATT_H
#define NIMBLE_GATT_H

#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"

void nimble_ble_task(void *pvParameters);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);
int gatt_svr_init(void);
void nimble_start(void);


#endif // NIMBLE_GATT_H
