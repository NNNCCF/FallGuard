# Wi-Fi Scan List on Provisioning Page

## Summary

Add a selectable Wi-Fi scan list to the bottom of the provisioning page served by the `wifi_manager` component. The page will automatically scan once after load, show nearby networks, and let the user tap a network to fill the SSID field. The password remains a manual input and the user still submits the existing connect form.

## Goal

Make Wi-Fi setup easier on mobile devices by reducing manual SSID typing while preserving the current connect flow and keeping hidden-network support through manual entry.

## Scope

In scope:

- Add a new HTTP endpoint that returns scanned Wi-Fi networks as JSON.
- Update the provisioning page UI to render the scanned network list under the form.
- Auto-trigger one scan on page load and provide a manual refresh control.
- Fill the SSID input when the user selects a scanned network.
- Keep the existing `/connect` and `/status` behavior unchanged.

Out of scope:

- Auto-connect when a network item is selected.
- Persisting scan results across reboots or page reloads.
- Advanced signal icons, sorting customization, or filtering controls.
- Changing the credential storage format or STA retry behavior.

## Existing Context

The current provisioning flow lives entirely in `components/wifi_manager/wifi_manager.c`.

- `/` serves a single embedded HTML page.
- `/status` returns device status JSON for polling.
- `/connect` accepts `ssid` and `password`, stores credentials, and starts the existing STA connection flow.
- When no valid credentials exist or connection retries fail, the device enters provisioning mode and exposes the provisioning page from its SoftAP.

This makes `wifi_manager` the right place to add both the scanning endpoint and the UI update, without changing application-level code.

One driver constraint shapes the solution: the ESP-IDF Wi-Fi API documents `esp_wifi_scan_start()` as supported in `WIFI_MODE_STA` or `WIFI_MODE_APSTA`, but not in `WIFI_MODE_AP`. Because of that, provisioning must keep the station interface enabled while the SoftAP page is active.

## Design

### Backend

Add a new `GET /scan` handler inside `wifi_manager`.

Behavior:

- Ensure provisioning mode runs in `WIFI_MODE_APSTA` so scanning is available while the SoftAP stays online.
- Trigger a synchronous Wi-Fi scan using `esp_wifi_scan_start(..., true)`.
- Read the scan results with `esp_wifi_scan_get_ap_records`.
- Return JSON with an array of visible networks.
- Include at least `ssid`, `rssi`, and a readable authentication label for each item.
- Exclude empty SSIDs from the response because tapping them would not help the user fill the form.

Response shape:

```json
{
  "ok": true,
  "networks": [
    {
      "ssid": "ExampleWiFi",
      "rssi": -48,
      "auth": "WPA2_PSK"
    }
  ]
}
```

Failure behavior:

- If scanning fails, return an HTTP 500 JSON response with `ok: false` and a short error message.
- If no networks are found, return `ok: true` with an empty array so the page can show an empty-state message.

Mode and safety assumptions:

- The scan endpoint is intended to be used while the device is serving the provisioning page in APSTA mode.
- The endpoint must not modify saved credentials or current status fields.
- A scan failure must not interrupt the existing provisioning page, AP mode, or manual SSID entry.
- The SoftAP credentials and HTTP server behavior remain unchanged; only the Wi-Fi mode used during provisioning is adjusted to support scanning.

### Frontend

Extend the embedded provisioning page with a scan section below the existing form and status card.

UI elements:

- A section title such as "Nearby Wi-Fi".
- A refresh button.
- A list container for network items.
- An inline message area for loading, empty, and error states.

Page behavior:

- Run one scan automatically after page load.
- Keep the existing status polling logic.
- Allow the user to refresh the Wi-Fi list manually.
- Render each network as a tappable row or button.
- When a row is selected, copy its SSID into the existing SSID input and focus the password input.

Compatibility:

- Manual SSID entry remains available at all times.
- Hidden networks remain supported because the SSID field is still editable.
- The connect action remains the existing form submission path.

### Data Flow

1. User joins the device SoftAP and opens the provisioning page.
2. The page loads and immediately requests `/scan`.
3. The device, already in APSTA provisioning mode, scans for nearby APs and returns a JSON list.
4. The page renders the list at the bottom of the form.
5. User taps a network item.
6. The page writes that SSID into the form input and focuses the password field.
7. User enters the password if needed and submits the existing connect form.
8. The existing `/connect` flow stores credentials and starts STA connection attempts.

## Error Handling

Backend:

- Reject internal scan failures with a JSON error response.
- Free any temporary buffers even when partial failures occur.
- Bound the number of AP records copied into memory to the scan result count reported by the Wi-Fi driver.
- If provisioning is not currently in a scan-capable mode, fail safely rather than interrupting the provisioning session.

Frontend:

- Show "Scanning nearby Wi-Fi..." while loading.
- Show "No Wi-Fi networks found." when the returned list is empty.
- Show a short retry message if `/scan` fails.
- Do not block the rest of the page when scanning fails.

## Testing

Manual verification:

- Start the device without valid Wi-Fi credentials and enter provisioning mode.
- Open the provisioning page from a phone or laptop.
- Confirm the page automatically shows nearby networks after load.
- Tap a listed network and verify the SSID field is filled while the password field remains editable.
- Press refresh and verify the list reloads.
- Submit valid credentials and confirm the existing connection flow still works.
- Confirm manual SSID entry still works when the desired network is not shown.

Code-level validation:

- Build the firmware with `idf.py build`.
- Confirm the new handler registration does not break the existing HTTP server startup.
- Check that JSON responses stay valid for normal, empty, and failure scan cases.

## Implementation Notes

- Keep all changes inside `components/wifi_manager/wifi_manager.c` unless a helper declaration becomes necessary.
- Reuse the existing JSON helper style and HTTP response patterns already used by `/status` and `/connect`.
- Prefer simple embedded JavaScript over larger UI restructuring so the page remains lightweight.
- Update provisioning-mode setup so the station interface stays enabled alongside the SoftAP.

## Acceptance Criteria

- The provisioning page shows a Wi-Fi list below the form.
- The page automatically scans once on load.
- The user can refresh the list manually.
- Selecting a listed network fills the SSID field only.
- The password is still entered manually and connection still requires pressing the existing connect button.
- Manual SSID entry remains available when scan results are missing or incomplete.
