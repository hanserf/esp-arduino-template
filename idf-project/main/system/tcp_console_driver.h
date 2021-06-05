#pragma once
typedef enum{
    CB_INFO,
    CB_WARNING,
    CB_ERROR
}cb_log_level_t;
typedef void (*log_callback_funtion_t)(cb_log_level_t level, const char *buf);
void register_log_callback(log_callback_funtion_t log_callback);
void start_console();
void tcp_console_socket_deinit();


