#include "ble_server.h"

static const char *ble_TAG = "BLE-Server";
uint8_t            ble_addr_type;

static void  ble_app_advertise(void);
static int   ble_gap_event(struct ble_gap_event *event, void *arg);
static int   device_write (uint16_t, uint16_t,
                           struct ble_gatt_access_ctxt *, void *);
static int   device_read  (uint16_t, uint16_t,
                           struct ble_gatt_access_ctxt *, void *);
static void  host_task    (void *param);

// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, 
                        uint16_t attr_handle, 
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);

    char buf[20];
    size_t len = (ctxt->om->om_len < sizeof(buf) - 1) ?
                  ctxt->om->om_len : sizeof(buf) - 1;
    memcpy(buf, ctxt->om->om_data, len);
    buf[len] = '\0';

    if      (!strcmp(buf, "LIGHT ON"))  ESP_LOGI(ble_TAG, "LIGHT ON");
    else if (!strcmp(buf, "LIGHT OFF")) ESP_LOGI(ble_TAG, "LIGHT OFF");
    else if (!strcmp(buf, "FAN ON"))    ESP_LOGI(ble_TAG, "FAN ON");
    else if (!strcmp(buf, "FAN OFF"))   ESP_LOGI(ble_TAG, "FAN OFF");
    else    ESP_LOGI(ble_TAG, "Data from client: %s", buf);

    return 0;
}

// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle,
                        uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const char *msg = "Data from the server";
    return os_mbuf_append(ctxt->om, msg, strlen(msg));
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}
};

// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(ble_TAG, "BLE GAP EVENT CONNECT %s", 
                event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise(); // reconnect
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(ble_TAG, "BLE GAP EVENT DISCONNECTED");
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(ble_TAG, "BLE GAP EVENT ADV COMPLETE - restart");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void ble_server_init(void)
{
    esp_err_t ret = nvs_flash_init(); // 1 - Initialize NVS flash
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    nimble_port_init();                        // Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Server"); // Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // Initialize application
    nimble_port_freertos_init(host_task);      // Run the thread

    ESP_LOGI(ble_TAG, "BLE server initialised!");
}