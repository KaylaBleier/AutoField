/**
 * AutoField UI - LVGL Implementation
 * 
 * Hardware: Arduino (with LVGL library)
 * Input: Keypad (4x3 or 4x4 matrix)
 * 
 * UI States:
 *   STATE_IDLE      - "Press # to start"
 *   STATE_USER_ID   - Enter User ID prompt
 *   STATE_RUNNING   - Running with progress bar
 * 
 * Keypad mapping:
 *   '#' = Start / Confirm
 *   '*' = Backspace / Cancel
 *   '0'-'9' = Numeric input
 */

#include "lvgl.h"
#include <string.h>
#include <stdio.h>

/* ── Palette ────────────────────────────────────────────── */
#define COLOR_BG        lv_color_hex(0x1A3A1A)   /* dark forest green */
#define COLOR_PANEL     lv_color_hex(0x0F2B0F)   /* slightly darker green */
#define COLOR_TERM_BG   lv_color_hex(0x000000)   /* terminal black */
#define COLOR_WHITE     lv_color_hex(0xFFFFFF)
#define COLOR_BORDER    lv_color_hex(0xFFFFFF)
#define COLOR_PROGRESS  lv_color_hex(0xFFFFFF)

/* ── Layout constants  (CrowPanel DIS08070H  800×480) ───── */
#define SCREEN_W        800
#define SCREEN_H        480

#define HEADER_H        80
#define HEADER_Y        14
#define HEADER_X        14

#define TERM_X          20
#define TERM_Y          115
#define TERM_W          420
#define TERM_H          330

#define INFO_X          465
#define INFO_Y          115
#define INFO_W          315
#define INFO_H          280

#define PROG_BAR_X      465
#define PROG_BAR_Y      420
#define PROG_BAR_W      315
#define PROG_BAR_H      38

#define INPUT_BUF_MAX   16

/* ── UI States ──────────────────────────────────────────── */
typedef enum {
    STATE_IDLE = 0,
    STATE_USER_ID,
    STATE_FIELD_WIDTH,
    STATE_FIELD_DEPTH,
    STATE_RUNNING
} ui_state_t;

/* ── Global UI objects ──────────────────────────────────── */
static lv_obj_t *scr;
static lv_obj_t *lbl_title;
static lv_obj_t *obj_terminal;
static lv_obj_t *lbl_terminal;
static lv_obj_t *lbl_user_id;
static lv_obj_t *lbl_field_width;
static lv_obj_t *lbl_field_depth;
static lv_obj_t *bar_progress;

/* ── Application state ──────────────────────────────────── */
static ui_state_t current_state = STATE_IDLE;
static char input_buf[INPUT_BUF_MAX] = {0};
static int  input_len = 0;
static char user_id[INPUT_BUF_MAX]    = {0};
static int  field_width = 0;
static int  field_depth = 0;

/* ══════════════════════════════════════════════════════════
 *  Style helpers
 * ══════════════════════════════════════════════════════════ */

static void apply_base_style(lv_obj_t *obj, lv_color_t bg, lv_color_t border, int radius)
{
    lv_obj_set_style_bg_color(obj, bg, 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(obj, border, 0);
    lv_obj_set_style_border_width(obj, 2, 0);
    lv_obj_set_style_border_opa(obj, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(obj, radius, 0);
    lv_obj_set_style_pad_all(obj, 8, 0);
}

static void apply_label_style(lv_obj_t *lbl, lv_color_t color, int font_size)
{
    /*
     * Using montserrat_14 everywhere - the only size enabled by default in lv_conf.h.
     * Once you enable larger fonts in lv_conf.h, restore the size-based selection:
     *   if (font_size >= 40) lv_obj_set_style_text_font(lbl, &lv_font_montserrat_40, 0);
     *   else if (font_size >= 28) lv_obj_set_style_text_font(lbl, &lv_font_montserrat_28, 0);
     *   else if (font_size >= 20) lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
     *   else lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
     * Also enable LV_FONT_UNSCII_16 in lv_conf.h for the terminal monospace font.
     */
    lv_obj_set_style_text_color(lbl, color, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
}

/* ══════════════════════════════════════════════════════════
 *  UI Construction
 * ══════════════════════════════════════════════════════════ */

void autofield_ui_init(void)
{
    /* ── Screen ─────────────────────────────────────────── */
    scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, COLOR_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* ── Outer card border ──────────────────────────────── */
    lv_obj_t *card = lv_obj_create(scr);
    lv_obj_set_size(card, SCREEN_W - 10, SCREEN_H - 10);
    lv_obj_set_pos(card, 5, 5);
    apply_base_style(card, COLOR_BG, COLOR_BORDER, 18);
    lv_obj_set_style_pad_all(card, 0, 0);

    /* ── Header bar ─────────────────────────────────────── */
    lv_obj_t *header = lv_obj_create(scr);
    lv_obj_set_size(header, SCREEN_W - 60, HEADER_H);
    lv_obj_set_pos(header, 30, HEADER_Y);
    apply_base_style(header, COLOR_PANEL, COLOR_BORDER, 30);

    lbl_title = lv_label_create(header);
    lv_label_set_text(lbl_title, "A u t o F i e l d");
    apply_label_style(lbl_title, COLOR_WHITE, 40);
    lv_obj_center(lbl_title);

    /* ── Terminal window ────────────────────────────────── */
    obj_terminal = lv_obj_create(scr);
    lv_obj_set_size(obj_terminal, TERM_W, TERM_H);
    lv_obj_set_pos(obj_terminal, TERM_X, TERM_Y);
    apply_base_style(obj_terminal, COLOR_TERM_BG, COLOR_BORDER, 10);
    lv_obj_set_style_pad_all(obj_terminal, 10, 0);

    lbl_terminal = lv_label_create(obj_terminal);
    lv_label_set_text(lbl_terminal, "Press # to start");
    lv_label_set_long_mode(lbl_terminal, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_terminal, TERM_W - 20);
    apply_label_style(lbl_terminal, COLOR_WHITE, 20);
    /* Use monospace font for terminal feel */
    lv_obj_set_style_text_font(lbl_terminal, &lv_font_montserrat_14, 0);  /* swap to &lv_font_unscii_16 once enabled in lv_conf.h */
    lv_obj_align(lbl_terminal, LV_ALIGN_TOP_LEFT, 0, 0);

    /* ── Info panel (right side) ────────────────────────── */
    /* User ID label */
    lbl_user_id = lv_label_create(scr);
    lv_label_set_text(lbl_user_id, "User ID:");
    apply_label_style(lbl_user_id, COLOR_WHITE, 20);
    lv_obj_set_pos(lbl_user_id, INFO_X, INFO_Y);

    /* Field Width label */
    lbl_field_width = lv_label_create(scr);
    lv_label_set_text(lbl_field_width, "Field Width:");
    apply_label_style(lbl_field_width, COLOR_WHITE, 20);
    lv_obj_set_pos(lbl_field_width, INFO_X, INFO_Y + 60);

    /* Field Depth label */
    lbl_field_depth = lv_label_create(scr);
    lv_label_set_text(lbl_field_depth, "Field Depth:");
    apply_label_style(lbl_field_depth, COLOR_WHITE, 20);
    lv_obj_set_pos(lbl_field_depth, INFO_X, INFO_Y + 120);

    /* "Progress" heading */
    lv_obj_t *lbl_prog_title = lv_label_create(scr);
    lv_label_set_text(lbl_prog_title, "Progress");
    apply_label_style(lbl_prog_title, COLOR_WHITE, 20);
    lv_obj_set_pos(lbl_prog_title, INFO_X, INFO_Y + 200);

    /* Progress bar */
    bar_progress = lv_bar_create(scr);
    lv_obj_set_size(bar_progress, PROG_BAR_W, PROG_BAR_H);
    lv_obj_set_pos(bar_progress, PROG_BAR_X, PROG_BAR_Y);
    lv_bar_set_range(bar_progress, 0, 100);
    lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);

    lv_obj_set_style_bg_color(bar_progress, COLOR_PANEL, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar_progress, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(bar_progress, COLOR_BORDER, LV_PART_MAIN);
    lv_obj_set_style_border_width(bar_progress, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(bar_progress, 14, LV_PART_MAIN);

    lv_obj_set_style_bg_color(bar_progress, COLOR_PROGRESS, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar_progress, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar_progress, 14, LV_PART_INDICATOR);
}

/* ══════════════════════════════════════════════════════════
 *  Terminal text helper (with blinking cursor line)
 * ══════════════════════════════════════════════════════════ */

static void terminal_print(const char *line1, const char *line2,
                            const char *line3, const char *cursor_line)
{
    char buf[256] = {0};
    if (line1)       { strncat(buf, line1,       sizeof(buf) - strlen(buf) - 1); }
    if (line2)       { strncat(buf, "\n",         sizeof(buf) - strlen(buf) - 1);
                       strncat(buf, line2,        sizeof(buf) - strlen(buf) - 1); }
    if (line3)       { strncat(buf, "\n",         sizeof(buf) - strlen(buf) - 1);
                       strncat(buf, line3,        sizeof(buf) - strlen(buf) - 1); }
    if (cursor_line) { strncat(buf, "\n",         sizeof(buf) - strlen(buf) - 1);
                       strncat(buf, cursor_line,  sizeof(buf) - strlen(buf) - 1); }
    lv_label_set_text(lbl_terminal, buf);
}

/* ══════════════════════════════════════════════════════════
 *  Update info panel labels
 * ══════════════════════════════════════════════════════════ */

static void update_info_labels(void)
{
    char buf[32];

    /* User ID */
    if (strlen(user_id) > 0) {
        snprintf(buf, sizeof(buf), "User ID: %s", user_id);
    } else {
        snprintf(buf, sizeof(buf), "User ID:");
    }
    lv_label_set_text(lbl_user_id, buf);

    /* Field Width */
    if (field_width > 0) {
        snprintf(buf, sizeof(buf), "Field Width:  %d ft", field_width);
    } else {
        snprintf(buf, sizeof(buf), "Field Width:");
    }
    lv_label_set_text(lbl_field_width, buf);

    /* Field Depth */
    if (field_depth > 0) {
        snprintf(buf, sizeof(buf), "Field Depth:  %d ft", field_depth);
    } else {
        snprintf(buf, sizeof(buf), "Field Depth:");
    }
    lv_label_set_text(lbl_field_depth, buf);
}

/* ══════════════════════════════════════════════════════════
 *  State machine — enter a new state
 * ══════════════════════════════════════════════════════════ */

static void enter_state(ui_state_t new_state)
{
    current_state = new_state;
    input_len = 0;
    memset(input_buf, 0, sizeof(input_buf));

    switch (new_state) {
        case STATE_IDLE:
            terminal_print("Press # to start", NULL, NULL, NULL);
            break;

        case STATE_USER_ID:
            terminal_print("Enter User ID",
                           "If you are a new user,",
                           "remember this for",
                           "repeat use ___");
            break;

        case STATE_FIELD_WIDTH:
            terminal_print("Enter Field Width", "(in feet)", NULL, "___");
            break;

        case STATE_FIELD_DEPTH:
            terminal_print("Enter Field Depth", "(in feet)", NULL, "___");
            break;

        case STATE_RUNNING:
            terminal_print("Running!", NULL, NULL, NULL);
            lv_bar_set_value(bar_progress, 0, LV_ANIM_OFF);
            break;
    }
    update_info_labels();
}

/* ══════════════════════════════════════════════════════════
 *  Build the "cursor line" showing current typed input
 * ══════════════════════════════════════════════════════════ */

static void redraw_input_line(void)
{
    char cursor_line[INPUT_BUF_MAX + 4] = {0};
    strncat(cursor_line, input_buf, sizeof(cursor_line) - 2);
    strncat(cursor_line, "_",       sizeof(cursor_line) - strlen(cursor_line) - 1);

    switch (current_state) {
        case STATE_USER_ID:
            terminal_print("Enter User ID",
                           "If you are a new user,",
                           "remember this for",
                           cursor_line);
            break;
        case STATE_FIELD_WIDTH:
            terminal_print("Enter Field Width", "(in feet)", NULL, cursor_line);
            break;
        case STATE_FIELD_DEPTH:
            terminal_print("Enter Field Depth", "(in feet)", NULL, cursor_line);
            break;
        default:
            break;
    }
}

/* ══════════════════════════════════════════════════════════
 *  Keypad handler  — call this from your keypad ISR / poll
 *
 *  key: '0'-'9', '#' (confirm/start), '*' (backspace/cancel)
 * ══════════════════════════════════════════════════════════ */

void autofield_key_handler(char key)
{
    switch (current_state) {

        /* ── IDLE: wait for '#' ─────────────────────────── */
        case STATE_IDLE:
            if (key == '#') {
                enter_state(STATE_USER_ID);
            }
            break;

        /* ── USER ID entry ──────────────────────────────── */
        case STATE_USER_ID:
            if (key >= '0' && key <= '9') {
                if (input_len < INPUT_BUF_MAX - 1) {
                    input_buf[input_len++] = key;
                    input_buf[input_len]   = '\0';
                    redraw_input_line();
                }
            } else if (key == '*') {
                /* backspace */
                if (input_len > 0) {
                    input_buf[--input_len] = '\0';
                    redraw_input_line();
                }
            } else if (key == '#') {
                if (input_len > 0) {
                    strncpy(user_id, input_buf, sizeof(user_id));
                    update_info_labels();
                    enter_state(STATE_FIELD_WIDTH);
                }
            }
            break;

        /* ── FIELD WIDTH entry ──────────────────────────── */
        case STATE_FIELD_WIDTH:
            if (key >= '0' && key <= '9') {
                if (input_len < 5) {   /* max 99999 ft  */
                    input_buf[input_len++] = key;
                    input_buf[input_len]   = '\0';
                    redraw_input_line();
                }
            } else if (key == '*') {
                if (input_len > 0) {
                    input_buf[--input_len] = '\0';
                    redraw_input_line();
                }
            } else if (key == '#') {
                if (input_len > 0) {
                    field_width = atoi(input_buf);
                    update_info_labels();
                    enter_state(STATE_FIELD_DEPTH);
                }
            }
            break;

        /* ── FIELD DEPTH entry ──────────────────────────── */
        case STATE_FIELD_DEPTH:
            if (key >= '0' && key <= '9') {
                if (input_len < 5) {
                    input_buf[input_len++] = key;
                    input_buf[input_len]   = '\0';
                    redraw_input_line();
                }
            } else if (key == '*') {
                if (input_len > 0) {
                    input_buf[--input_len] = '\0';
                    redraw_input_line();
                }
            } else if (key == '#') {
                if (input_len > 0) {
                    field_depth = atoi(input_buf);
                    update_info_labels();
                    enter_state(STATE_RUNNING);
                }
            }
            break;

        /* ── RUNNING: '#' to reset, '*' to abort ────────── */
        case STATE_RUNNING:
            if (key == '*') {
                /* Reset everything */
                memset(user_id, 0, sizeof(user_id));
                field_width = 0;
                field_depth = 0;
                update_info_labels();
                enter_state(STATE_IDLE);
            }
            break;
    }
}

/* ══════════════════════════════════════════════════════════
 *  Progress bar update  — call from your task/timer
 *  value: 0-100
 * ══════════════════════════════════════════════════════════ */

void autofield_set_progress(int value)
{
    if (value < 0)   value = 0;
    if (value > 100) value = 100;
    lv_bar_set_value(bar_progress, value, LV_ANIM_ON);
}

/* ══════════════════════════════════════════════════════════
 *  Getters (call from your control logic)
 * ══════════════════════════════════════════════════════════ */

const char *autofield_get_user_id(void)    { return user_id; }
int         autofield_get_field_width(void) { return field_width; }
int         autofield_get_field_depth(void) { return field_depth; }
ui_state_t  autofield_get_state(void)      { return current_state; }
int         autofield_is_running(void)     { return current_state == STATE_RUNNING; }