/*!
 * @file StepScreenLayout.h
 *
 * Screen zone constants for the 128x64 SH1106 panel.
 *
 *   x=0                 x=100   x=127
 *   +---------------------+-------+  y=0
 *   | Info bar            |  Back |  y=0..7
 *   +---------------------+-------+  y=8
 *   |                     |       |
 *   | Content             |  Sel  |  (encoder push, y=26)
 *   |                     |       |
 *   |                     |   OK  |  (confirm, y=52)
 *   +---------------------+-------+  y=63
 *
 * The action column holds right-justified labels for the three physical
 * buttons: Back (top), encoder push "Sel" (middle), Confirm "OK" (bottom).
 */

#ifndef STEPSCREEN_LAYOUT_H
#define STEPSCREEN_LAYOUT_H

#define STEPSCREEN_W 128
#define STEPSCREEN_H 64

// Top info bar (spans the content width; the action column keeps the
// Back label on the same row)
#define STEPSCREEN_INFO_BAR_H 8

// Right-justified action column
#define STEPSCREEN_ACTION_COL_X 100
#define STEPSCREEN_ACTION_COL_W 28

// Main content area (left of the action column, below the info bar)
#define STEPSCREEN_CONTENT_X 0
#define STEPSCREEN_CONTENT_Y 8
#define STEPSCREEN_CONTENT_W 100
#define STEPSCREEN_CONTENT_H 56

// Vertical anchors for the action column labels (8px font rows)
#define STEPSCREEN_BTN_BACK_Y 0
#define STEPSCREEN_BTN_PUSH_Y 26
#define STEPSCREEN_BTN_CONFIRM_Y 52

#endif // STEPSCREEN_LAYOUT_H
