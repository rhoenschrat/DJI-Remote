#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

#include "dji_protocol_data_descriptors.h"
#include "dji_protocol_data_structures.h"

#define TAG "DJI_PROTOCOL_DATA_DESCRIPTORS"

/* Structure support, but need to define creator and parser for each structure */
const data_descriptor_t data_descriptors[] = {
    // Camera mode switch
    {0x1D, 0x04, (data_creator_func_t)camera_mode_switch_creator, (data_parser_func_t)camera_mode_switch_parser},
    // Version query
    {0x00, 0x00, NULL, (data_parser_func_t)version_query_parser},
    // Record control
    {0x1D, 0x03, (data_creator_func_t)record_control_creator, (data_parser_func_t)record_control_parser},
    // Connection request (handshake)
    {0x00, 0x19, (data_creator_func_t)connection_data_creator, (data_parser_func_t)connection_data_parser},
    // GPS data push (CmdSet=0x00, CmdID=0x17 - DJI camera protocol for GPS telemetry)
    {0x00, 0x17, (data_creator_func_t)gps_data_push_creator, NULL},
    // Camera status subscription (CmdSet=0x1D, CmdID=0x05)
    {0x1D, 0x05, (data_creator_func_t)camera_status_subscription_creator, NULL},
    // Camera status push
    {0x1D, 0x02, NULL, (data_parser_func_t)camera_status_push_data_parser},
    // New Camera status push
    {0x1D, 0x06, NULL, (data_parser_func_t)new_camera_status_push_data_parser},
    // Key report
    {0x00, 0x11, (data_creator_func_t)key_report_creator, (data_parser_func_t)key_report_parser},
    // Camera power mode switch
    {0x00, 0x1A, (data_creator_func_t)camera_power_mode_switch_creator, (data_parser_func_t)camera_power_mode_switch_parser},
};
const size_t DATA_DESCRIPTORS_COUNT = sizeof(data_descriptors) / sizeof(data_descriptors[0]);

/* Structure support creators and parsers */
uint8_t* camera_mode_switch_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    // Check if it's a command frame
    if ((cmd_type & 0x20) == 0) {
        const camera_mode_switch_command_frame_t *command_frame = 
            (const camera_mode_switch_command_frame_t *)structure;

        *data_length = sizeof(camera_mode_switch_command_frame_t);
        
        ESP_LOGI(TAG, "Data length calculated for camera_mode_switch_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in camera_mode_switch_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for command frame, copying data...");
        
        memcpy(data, command_frame, *data_length);
    } else {
        // Response frame creation for this functionality is not yet supported.
        ESP_LOGE(TAG, "Response frames are not supported in camera_mode_switch_creator");
        return NULL;
    }

    return data;
}

int camera_mode_switch_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "camera_mode_switch_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Camera Mode Switch data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        // Command frame parsing for this functionality is not yet supported.
        ESP_LOGE(TAG, "camera_mode_switch_parser: Only response frames are supported");
        return -1;
    }

    if (data_length < sizeof(camera_mode_switch_response_frame_t)) {
        ESP_LOGE(TAG, "camera_mode_switch_parser: Data length too short for response frame. Expected: %zu, Got: %zu",
                 sizeof(camera_mode_switch_response_frame_t), data_length);
        return -1;
    }

    const camera_mode_switch_response_frame_t *response = (const camera_mode_switch_response_frame_t *)data;
    camera_mode_switch_response_frame_t *output_response = (camera_mode_switch_response_frame_t *)structure_out;

    output_response->ret_code = response->ret_code;
    memcpy(output_response->reserved, response->reserved, sizeof(response->reserved));

    ESP_LOGI(TAG, "Camera Mode Switch Response parsed successfully. ret_code: %u", output_response->ret_code);

    return 0;
}

int version_query_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "version_query_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Version Query Response, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        ESP_LOGE(TAG, "version_query_parser: Only response frames are supported");
        return -1;
    }

    // ack_result(2 bytes) + product_id(16 bytes)
    size_t fixed_length = sizeof(uint16_t) + 16;
    if (data_length < fixed_length) {
        ESP_LOGE(TAG, "version_query_parser: Data length too short for response frame. Expected at least: %zu, Got: %zu",
                 fixed_length, data_length);
        return -1;
    }

    // Calculate flexible array part length
    size_t sdk_version_length = data_length - fixed_length;

    // Ensure structure_out has enough space
    version_query_response_frame_t *output_response = (version_query_response_frame_t *)structure_out;

    // Fill fixed part
    output_response->ack_result = *(const uint16_t *)data;
    memcpy(output_response->product_id, data + sizeof(uint16_t), 16);

    // Fill flexible array part
    memcpy(output_response->sdk_version, data + fixed_length, sdk_version_length);

    ESP_LOGI(TAG, "Version Query Response parsed successfully. ack_result: %u, product_id: %s, sdk_version: %.*s",
             output_response->ack_result, output_response->product_id, (int)sdk_version_length, output_response->sdk_version);

    return 0;
}

uint8_t* record_control_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    if ((cmd_type & 0x20) == 0) {
        const record_control_command_frame_t *command_frame = 
            (const record_control_command_frame_t *)structure;

        *data_length = sizeof(record_control_command_frame_t);
        
        ESP_LOGI(TAG, "Data length calculated for record_control_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in record_control_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for command frame, copying data...");
        
        memcpy(data, command_frame, *data_length);
    } else {
        ESP_LOGE(TAG, "Response frames are not supported in record_control_creator");
        return NULL;
    }

    return data;
}

int record_control_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "record_control_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Record Control Response, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        ESP_LOGW(TAG, "record_control_parser: Parsing command frame is not supported.");
        return -1;
    }

    if (data_length < sizeof(record_control_response_frame_t)) {
        ESP_LOGE(TAG, "record_control_parser: Data length too short. Expected: %zu, Got: %zu",
                 sizeof(record_control_response_frame_t), data_length);
        return -1;
    }

    const record_control_response_frame_t *response = (const record_control_response_frame_t *)data;

    record_control_response_frame_t *output_frame = (record_control_response_frame_t *)structure_out;
    output_frame->ret_code = response->ret_code;

    ESP_LOGI(TAG, "Record Control Response parsed successfully. ret_code: %d", output_frame->ret_code);

    return 0;
}


uint8_t* connection_data_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "connection_request_data_creator: NULL input detected");
        return NULL;
    }

    uint8_t *data = NULL;

    if ((cmd_type & 0x20) == 0) {
        const connection_request_command_frame *command_frame = (const connection_request_command_frame *)structure;

        *data_length = sizeof(connection_request_command_frame);

        ESP_LOGI(TAG, "Data length calculated for connection_request_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in connection_request_data_creator (command frame)");
            return NULL;
        }

        memcpy(data, command_frame, *data_length);
    } else {
        const connection_request_response_frame *response_frame = (const connection_request_response_frame *)structure;

        *data_length = sizeof(connection_request_response_frame);

        ESP_LOGI(TAG, "Data length calculated for connection_request_response_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in connection_request_data_creator (response frame)");
            return NULL;
        }

        memcpy(data, response_frame, *data_length);
    }

    return data;
}

int connection_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "connection_request_data_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Connection Request data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        ESP_LOGI(TAG, "Parsing command frame...");

        if (data_length < sizeof(connection_request_command_frame)) {
            ESP_LOGE(TAG, "connection_request_data_parser: Data length too short for command frame. Expected: %zu, Got: %zu",
                     sizeof(connection_request_command_frame), data_length);
            return -1;
        }

        const connection_request_command_frame *command = (const connection_request_command_frame *)data;

        connection_request_command_frame *output_command = (connection_request_command_frame *)structure_out;
        output_command->device_id = command->device_id;
        output_command->mac_addr_len = command->mac_addr_len;
        memcpy(output_command->mac_addr, command->mac_addr, sizeof(command->mac_addr));
        output_command->fw_version = command->fw_version;
        output_command->conidx = command->conidx;
        output_command->verify_mode = command->verify_mode;
        output_command->verify_data = command->verify_data;
        memcpy(output_command->reserved, command->reserved, sizeof(command->reserved));

        return 0;
    } else {
        ESP_LOGI(TAG, "Parsing response frame...");

        if (data_length < sizeof(connection_request_response_frame)) {
            ESP_LOGE(TAG, "connection_request_data_parser: Data length too short for response frame. Expected: %zu, Got: %zu",
                     sizeof(connection_request_response_frame), data_length);
            return -1;
        }

        const connection_request_response_frame *response = (const connection_request_response_frame *)data;

        connection_request_response_frame *output_response = (connection_request_response_frame *)structure_out;
        output_response->device_id = response->device_id;
        output_response->ret_code = response->ret_code;
        memcpy(output_response->reserved, response->reserved, sizeof(response->reserved));

        return 0;
    }
}

uint8_t* camera_status_subscription_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    if ((cmd_type & 0x20) == 0) {
        const camera_status_subscription_command_frame *camera_command_frame = 
            (const camera_status_subscription_command_frame *)structure;

        *data_length = sizeof(camera_status_subscription_command_frame);
        
        ESP_LOGI(TAG, "Data length calculated for camera_status_subscription_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in camera_status_subscription_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for command frame, copying data...");
        
        memcpy(data, camera_command_frame, *data_length);
    } else {
        ESP_LOGE(TAG, "Response frames are not supported in camera_status_subscription_creator");
        return NULL;
    }
    return data;
}

int camera_status_push_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "camera_status_push_data_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Camera Status Push data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        if (data_length < sizeof(camera_status_push_command_frame)) {
            ESP_LOGE(TAG, "camera_status_push_data_parser: Data length too short for command frame. Expected: %zu, Got: %zu",
                     sizeof(camera_status_push_command_frame), data_length);
            return -1;
        }

        const camera_status_push_command_frame *frame = (const camera_status_push_command_frame *)data;

        camera_status_push_command_frame *output_frame = (camera_status_push_command_frame *)structure_out;
        output_frame->camera_mode = frame->camera_mode;
        output_frame->camera_status = frame->camera_status;
        output_frame->video_resolution = frame->video_resolution;
        output_frame->fps_idx = frame->fps_idx;
        output_frame->eis_mode = frame->eis_mode;
        output_frame->record_time = frame->record_time;
        output_frame->fov_type = frame->fov_type;
        output_frame->photo_ratio = frame->photo_ratio;
        output_frame->real_time_countdown = frame->real_time_countdown;
        output_frame->timelapse_interval = frame->timelapse_interval;
        output_frame->timelapse_duration = frame->timelapse_duration;
        output_frame->remain_capacity = frame->remain_capacity;
        output_frame->remain_photo_num = frame->remain_photo_num;
        output_frame->remain_time = frame->remain_time;
        output_frame->user_mode = frame->user_mode;
        output_frame->power_mode = frame->power_mode;
        output_frame->camera_mode_next_flag = frame->camera_mode_next_flag;
        output_frame->temp_over = frame->temp_over;
        output_frame->photo_countdown_ms = frame->photo_countdown_ms;
        output_frame->loop_record_sends = frame->loop_record_sends;
        output_frame->camera_bat_percentage = frame->camera_bat_percentage;

        return 0;
    } else {
        ESP_LOGE(TAG, "camera_status_push_data_parser: Response frames are not supported");
        return -1;
    }
}

int new_camera_status_push_data_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "new_camera_status_push_data_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing New Camera Status Push data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        // DJI 1D06 frame format (fixed-size fields, padded with zeros):
        // - type_mode_name (1 byte): 0x01
        // - mode_name_length (1 byte): actual string length (1-21)
        // - mode_name (21 bytes): null-padded string
        // - type_mode_param (1 byte): 0x02
        // - mode_param_length (1 byte): actual string length (1-21)
        // - mode_param (21 bytes): null-padded string
        // Total: 46 bytes for DATA section
        //
        // Wire format offsets:
        //   Offset  0: type_mode_name (0x01)
        //   Offset  1: mode_name_length
        //   Offset  2-22: mode_name (21 bytes, zero-padded)
        //   Offset 23: type_mode_param (0x02)
        //   Offset 24: mode_param_length
        //   Offset 25-45: mode_param (21 bytes, zero-padded)
        
        // Minimum size: type(1) + length(1) + data(21) + type(1) + length(1) + data(21) = 46 bytes
        if (data_length < 46) {
            ESP_LOGE(TAG, "new_camera_status_push_data_parser: Data length too short. Expected 46, got: %zu", data_length);
            return -1;
        }

        new_camera_status_push_command_frame *output_frame = (new_camera_status_push_command_frame *)structure_out;
        memset(output_frame, 0, sizeof(new_camera_status_push_command_frame));
        
        // Parse mode_name section (fixed offsets)
        // Offset 0: type_mode_name
        // Offset 1: mode_name_length
        // Offset 2-22: mode_name data (21 bytes, padded with zeros)
        output_frame->type_mode_name = data[0];
        output_frame->mode_name_length = data[1];
        
        // Validate mode_name_length (max 21 bytes in wire format, but our struct has 20-byte array + null)
        if (output_frame->mode_name_length > 20) {
            ESP_LOGW(TAG, "new_camera_status_push_data_parser: mode_name_length (%d) exceeds 20, truncating", output_frame->mode_name_length);
            output_frame->mode_name_length = 20;
        }
        
        // Copy mode_name (actual string bytes only, not the padding)
        if (output_frame->mode_name_length > 0) {
            memcpy(output_frame->mode_name, &data[2], output_frame->mode_name_length);
        }
        output_frame->mode_name[output_frame->mode_name_length] = '\0';  // Null-terminate
        
        // Parse mode_param section (fixed offsets)
        // Offset 23: type_mode_param
        // Offset 24: mode_param_length
        // Offset 25-45: mode_param data (21 bytes, padded with zeros)
        output_frame->type_mode_param = data[23];
        output_frame->mode_param_length = data[24];
        
        // Validate mode_param_length (max 21 bytes in wire format, but our struct has 20-byte array + null)
        if (output_frame->mode_param_length > 20) {
            ESP_LOGW(TAG, "new_camera_status_push_data_parser: mode_param_length (%d) exceeds 20, truncating", output_frame->mode_param_length);
            output_frame->mode_param_length = 20;
        }
        
        // Copy mode_param (actual string bytes only, not the padding)
        if (output_frame->mode_param_length > 0) {
            memcpy(output_frame->mode_param, &data[25], output_frame->mode_param_length);
        }
        output_frame->mode_param[output_frame->mode_param_length] = '\0';  // Null-terminate

        ESP_LOGI(TAG, "new_camera_status_push_data_parser: Parsed mode_name='%s' (len=%d), mode_param='%s' (len=%d)",
                 output_frame->mode_name, output_frame->mode_name_length,
                 output_frame->mode_param, output_frame->mode_param_length);

        return 0;
    } else {
        ESP_LOGE(TAG, "new_camera_status_push_data_parser: Response frames are not supported");
        return -1;
    }
}

uint8_t* key_report_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    if ((cmd_type & 0x20) == 0) {
        const key_report_command_frame_t *command_frame = 
            (const key_report_command_frame_t *)structure;

        *data_length = sizeof(key_report_command_frame_t);
        
        ESP_LOGI(TAG, "Data length calculated for key_report_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in key_report_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for command frame, copying data...");
        
        memcpy(data, command_frame, *data_length);
    } else {
        ESP_LOGE(TAG, "Response frames are not supported in key_report_creator");
        return NULL;
    }

    return data;
}

int key_report_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "key_report_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Key Report Response data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        ESP_LOGE(TAG, "key_report_parser: Only response frames are supported");
        return -1;
    }

    if (data_length < sizeof(key_report_response_frame_t)) {
        ESP_LOGE(TAG, "key_report_parser: Data length too short for response frame. Expected: %zu, Got: %zu",
                 sizeof(key_report_response_frame_t), data_length);
        return -1;
    }

    const key_report_response_frame_t *response = (const key_report_response_frame_t *)data;
    key_report_response_frame_t *output_response = (key_report_response_frame_t *)structure_out;

    output_response->ret_code = response->ret_code;

    ESP_LOGI(TAG, "Key Report Response parsed successfully. ret_code: %u", output_response->ret_code);

    return 0;
}

uint8_t* camera_power_mode_switch_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    // Check if it's a command frame
    if ((cmd_type & 0x20) == 0) {
        const camera_power_mode_switch_command_frame_t *command_frame = 
            (const camera_power_mode_switch_command_frame_t *)structure;

        *data_length = sizeof(camera_power_mode_switch_command_frame_t);

        ESP_LOGI(TAG, "Data length calculated for camera_power_mode_switch_command_frame: %zu", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in camera_power_mode_switch_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for command frame, copying data...");

        memcpy(data, command_frame, *data_length);
    } else {
        // Response frame creation for this functionality is not yet supported.
        ESP_LOGE(TAG, "Response frames are not supported in camera_power_mode_switch_creator");
        return NULL;
    }

    return data;
}

int camera_power_mode_switch_parser(const uint8_t *data, size_t data_length, void *structure_out, uint8_t cmd_type) {
    if (data == NULL || structure_out == NULL) {
        ESP_LOGE(TAG, "camera_power_mode_switch_parser: NULL input detected");
        return -1;
    }

    ESP_LOGI(TAG, "Parsing Camera Power Mode Switch data, received data length: %zu", data_length);

    if ((cmd_type & 0x20) == 0) {
        // Command frame parsing for this functionality is not yet supported.
        ESP_LOGE(TAG, "camera_power_mode_switch_parser: Only response frames are supported");
        return -1;
    }

    if (data_length < sizeof(camera_power_mode_switch_response_frame_t)) {
        ESP_LOGE(TAG, "camera_power_mode_switch_parser: Data length too short for response frame. Expected: %zu, Got: %zu",
                 sizeof(camera_power_mode_switch_response_frame_t), data_length);
        return -1;
    }

    const camera_power_mode_switch_response_frame_t *response = (const camera_power_mode_switch_response_frame_t *)data;
    camera_power_mode_switch_response_frame_t *output_response = (camera_power_mode_switch_response_frame_t *)structure_out;

    output_response->ret_code = response->ret_code;

    ESP_LOGI(TAG, "Camera Power Mode Switch Response parsed successfully. ret_code: %u", output_response->ret_code);

    return 0;
}

/**
 * @brief GPS data push creator
 * 
 * Creates GPS data push frame for DJI protocol (CmdSet=0x00, CmdID=0x17).
 * Payload size is ~45 bytes, total frame size with protocol headers is ~66 bytes.
 * This matches the format used by working iOS implementations.
 */
uint8_t* gps_data_push_creator(const void *structure, size_t *data_length, uint8_t cmd_type) {
    if (structure == NULL || data_length == NULL) {
        ESP_LOGE(TAG, "Invalid input: structure or data_length is NULL");
        return NULL;
    }

    uint8_t *data = NULL;

    // GPS data push is always a command frame (no response)
    if ((cmd_type & 0x20) == 0) {
        const gps_data_push_command_frame_t *command_frame = 
            (const gps_data_push_command_frame_t *)structure;

        *data_length = sizeof(gps_data_push_command_frame_t);

        ESP_LOGI(TAG, "GPS data push frame created: payload_size=%zu bytes (CmdSet=0x00, CmdID=0x17)", *data_length);

        data = (uint8_t *)malloc(*data_length);
        if (data == NULL) {
            ESP_LOGE(TAG, "Memory allocation failed in gps_data_push_creator");
            return NULL;
        }

        ESP_LOGI(TAG, "Memory allocation succeeded for GPS data push frame, copying data...");

        memcpy(data, command_frame, *data_length);
    } else {
        ESP_LOGE(TAG, "Response frames are not supported in gps_data_push_creator");
        return NULL;
    }

    return data;
}