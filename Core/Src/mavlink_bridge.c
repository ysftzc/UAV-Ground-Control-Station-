/* MAVLink v2 koprusu - bkz. mavlink_bridge.h */
#include "mavlink_bridge.h"
#include "common/mavlink.h"

/* Bu kart PX4'un kendisi degil, ona sensor verisi sağlayan ayri bir
 * donanım (GY-87 + STM32) - component ID olarak otopilotun kendisini
 * degil, MAV_COMP_ID_PERIPHERAL'i kullanmak PX4/QGC konvansiyonuyla
 * tutarli (parametre mikroservisi implemente etmeyen yardimci cihazlar
 * icin ayrilmis ID). */
#define MAVLINK_BRIDGE_SYSTEM_ID    1
#define MAVLINK_BRIDGE_COMPONENT_ID MAV_COMP_ID_PERIPHERAL

#define MAVLINK_BRIDGE_G_TO_MS2    9.80665f
#define MAVLINK_BRIDGE_DEG_TO_RAD  0.017453293f
#define MAVLINK_BRIDGE_UT_TO_GAUSS 0.01f

static void MAVLink_Send(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                          const mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        HAL_UART_Transmit(huart, buf, len, 100);
        xSemaphoreGive(uart_mutex);
    }
}

void MAVLink_SendHeartbeat(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex) {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(MAVLINK_BRIDGE_SYSTEM_ID, MAVLINK_BRIDGE_COMPONENT_ID, &msg,
                                MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
                                0, 0, MAV_STATE_ACTIVE);

    MAVLink_Send(huart, uart_mutex, &msg);
}

void MAVLink_SendHilSensor(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                            const MAVLink_SensorSnapshot_t *snapshot, uint64_t time_usec) {
    mavlink_message_t msg;
    uint32_t fields_updated = HIL_SENSOR_UPDATED_XACC | HIL_SENSOR_UPDATED_YACC | HIL_SENSOR_UPDATED_ZACC |
                              HIL_SENSOR_UPDATED_XGYRO | HIL_SENSOR_UPDATED_YGYRO | HIL_SENSOR_UPDATED_ZGYRO |
                              HIL_SENSOR_UPDATED_XMAG | HIL_SENSOR_UPDATED_YMAG | HIL_SENSOR_UPDATED_ZMAG |
                              HIL_SENSOR_UPDATED_ABS_PRESSURE | HIL_SENSOR_UPDATED_PRESSURE_ALT |
                              HIL_SENSOR_UPDATED_TEMPERATURE;

    mavlink_msg_hil_sensor_pack(MAVLINK_BRIDGE_SYSTEM_ID, MAVLINK_BRIDGE_COMPONENT_ID, &msg,
        time_usec,
        snapshot->accel_x * MAVLINK_BRIDGE_G_TO_MS2,
        snapshot->accel_y * MAVLINK_BRIDGE_G_TO_MS2,
        snapshot->accel_z * MAVLINK_BRIDGE_G_TO_MS2,
        snapshot->gyro_x * MAVLINK_BRIDGE_DEG_TO_RAD,
        snapshot->gyro_y * MAVLINK_BRIDGE_DEG_TO_RAD,
        snapshot->gyro_z * MAVLINK_BRIDGE_DEG_TO_RAD,
        snapshot->mag_x * MAVLINK_BRIDGE_UT_TO_GAUSS,
        snapshot->mag_y * MAVLINK_BRIDGE_UT_TO_GAUSS,
        snapshot->mag_z * MAVLINK_BRIDGE_UT_TO_GAUSS,
        snapshot->pressure, 0.0f /* diff_pressure - yok */,
        snapshot->altitude, snapshot->temperature,
        fields_updated, 0 /* id - tek IMU */);

    MAVLink_Send(huart, uart_mutex, &msg);
}

static void MAVLink_SendNamedValueInt(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                                       uint32_t time_boot_ms, const char *name, int32_t value) {
    mavlink_message_t msg;
    mavlink_msg_named_value_int_pack(MAVLINK_BRIDGE_SYSTEM_ID, MAVLINK_BRIDGE_COMPONENT_ID, &msg,
                                      time_boot_ms, name, value);
    MAVLink_Send(huart, uart_mutex, &msg);
}

void MAVLink_SendTaskHealth(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                             const MAVLink_TaskHealth_t *health, uint32_t time_boot_ms) {
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_imu",  (int32_t)health->imu_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_baro", (int32_t)health->baro_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_mag",  (int32_t)health->mag_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_can",  (int32_t)health->can_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_mav",  (int32_t)health->mav_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "hwm_wdg",  (int32_t)health->wdg_hwm);
    MAVLink_SendNamedValueInt(huart, uart_mutex, time_boot_ms, "heap",     (int32_t)health->heap_free);
}
