# Wi-Fi SSID Right-Top Overlay Design

## 1. Goal

Display the currently connected Wi-Fi SSID at the top-right corner of the OLED on **all pages**.
If the SSID text is too long, show it with a marquee animation.

## 2. Confirmed Requirements

- Scope: apply to all OLED pages (overview, params, cloud scene).
- Position: top-right corner.
- Overflow behavior: marquee scrolling.
- Not connected: show `NO-WIFI`.
- Keep existing page content and switching behavior unchanged.

## 3. Recommended Approach

Use a **unified overlay layer** in `oled_display_render_snapshot()`:

- Render page content first (existing flow).
- Render Wi-Fi SSID overlay last (new overlay function).
- Keep all page-specific render functions unaware of SSID details.

Why this approach:

- Single implementation point, low duplication.
- Automatically applies to current and future pages.
- Lower maintenance cost than adding SSID logic into each page renderer.

## 4. Architecture Changes

### 4.1 OLED Snapshot Extension

Extend `oled_display_snapshot_t` with Wi-Fi display data:

- `bool wifi_connected;`
- `char wifi_ssid[33];` (same max length as `wifi_status_t.ssid`)
- marquee state fields (offset, timing, pause bookkeeping).

### 4.2 OLED API Extension

Add one update API in `oled_display.h`:

- `esp_err_t oled_display_set_wifi_status(const char *ssid, bool connected);`

Implementation pattern:

- Reuse current `oled_display_update_snapshot(...)` and lock-protected update callbacks.
- Sanitize input SSID before storing (replace unsupported glyphs with space).

### 4.3 Main Loop Integration

In `main.c`, periodically call `wifi_manager_get_status(...)` and push data:

- If state is connected and `ssid` non-empty: `connected=true` and pass SSID.
- Otherwise: `connected=false`, SSID ignored or cleared.

This runs in the existing main loop and does not alter button flow.

## 5. Rendering and Marquee Behavior

### 5.1 Overlay Placement

- Overlay is rendered on line 0 in the right-side area.
- Right side has priority; left page title remains in place.
- Drawing must be clipped to the right-side overlay area to avoid polluting left text.

### 5.2 Text Rules

- Connected + short text: static right-aligned display.
- Connected + long text: marquee scroll leftward across the viewport.
- Disconnected: static `NO-WIFI`.

### 5.3 Animation Timing

- Step interval: 200 ms.
- Step size: 1 pixel per interval.
- End pause after full pass: 800 ms.
- On SSID change or connect transition: reset marquee to start position immediately.

## 6. Error Handling and Robustness

- If `wifi_manager_get_status(...)` fails once: keep last rendered Wi-Fi overlay state.
- If repeated failures exceed a small threshold: fallback to disconnected display (`NO-WIFI`).
- Always guard shared snapshot data with existing mutex logic.
- Keep rendering deterministic: overlay function should not allocate memory dynamically.

## 7. Performance Notes

- Keep OLED refresh cadence unchanged.
- Compute marquee state with lightweight arithmetic only.
- Optional optimization: redraw overlay region only when text/offset changes.

## 8. Validation Plan

Manual checks:

1. Connected with short SSID: top-right shows full SSID statically on all pages.
2. Connected with long SSID: smooth marquee and 800 ms end pause on all pages.
3. Wi-Fi disconnect/reconnect: immediate transition between `NO-WIFI` and SSID.
4. Page switching and cloud rendering remain normal.
5. No regressions in button short/long/factory-reset behavior.

## 9. Out of Scope

- New iconography (signal bars, lock icon).
- Top status bar redesign that shifts all page content downward.
- Multi-line SSID rendering.
