#ifndef NIMBLE_GATT_H
#define NIMBLE_GATT_H

#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

/* NimBLE GAP APIs */
#include "host/ble_gap.h"

void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_subscribe_cb(struct ble_gap_event *event);
int gatt_init(void);

// temp
void imu_data_task(void *param);
int gatt_send_imu_notification(const uint8_t *data, size_t len);

#endif // NIMBLE_GATT_H
