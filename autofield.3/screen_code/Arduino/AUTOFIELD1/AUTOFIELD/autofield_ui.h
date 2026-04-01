#ifndef AUTOFIELD_UI_H
#define AUTOFIELD_UI_H

#include "lvgl.h"

/* UI States */
typedef enum {
    STATE_IDLE = 0,
    STATE_USER_ID,
    STATE_FIELD_WIDTH,
    STATE_FIELD_DEPTH,
    STATE_RUNNING
} ui_state_t;

/* Initialize all LVGL widgets — call once after lv_init() */
void autofield_ui_init(void);

/**
 * Feed a keypad character into the state machine.
 * key: '0'-'9'  numeric input
 *      '#'       confirm / start
 *      '*'       backspace / cancel / abort
 */
void autofield_key_handler(char key);

/**
 * Update the progress bar.
 * value: 0-100
 */
void autofield_set_progress(int value);

/* Getters */
const char *autofield_get_user_id(void);
int         autofield_get_field_width(void);
int         autofield_get_field_depth(void);
ui_state_t  autofield_get_state(void);
int         autofield_is_running(void);

#endif /* AUTOFIELD_UI_H */