# DJI-Remote – User Manual

This manual explains how to operate the DJI-Remote device, covering startup behavior, pairing, single-camera control, multi-camera coordination, and GPS functionality.

> This manual applies to both supported hardware targets: **M5Stack Basic V2.7**
> (ESP32, built-in buttons) and **Waveshare ESP32-S3-LCD-1.9** (ESP32-S3,
> external buttons wired to GPIO5/6/16). All operations are identical on both
> boards.


## Using the Buttons

The remote has three buttons, **A**, **B** and **C**. Their function depends on
the screen and is shown in the bottom row of the display.

- On the **Main Screen**, a button reacts when it is **released**, because
  each button also has a **long-press** function there (see below). On all
  other screens, a button reacts as soon as it is pressed.
- Every press is checked for about 0.1 s, which filters out contact bounce
  and electrical glitches.
- Holding a button does **not** repeat the action and does not stop the
  remote: the display, the recording timer and the camera connections keep
  running while a button is held.
- You can press buttons quickly one after another (e.g. **Next**): every
  press counts, there is no waiting time after releasing a button.
- A button that is already pressed when the remote starts is ignored until it
  has been released once.
- A button that is held for more than 10 seconds (e.g. squeezed in a bag)
  triggers nothing further until it is released.
- **External buttons (M5Stack, optional):** buttons wired to G26 (A), G21 (B)
  and G22 (C) behave exactly like the built-in buttons on every screen,
  including long press. There is no extra delay or cooldown for them.

### Long Press on the Main Screen: Record on One Camera

Hold a button for **1 second** to control one camera directly, without
changing the camera selection:

| Hold | Camera |
|------|--------|
| **Button A** | Camera 1 (left) |
| **Button B** | Camera 2 (middle) |
| **Button C** | Camera 3 (right) |

The action happens as soon as 1 s is reached; releasing the button
afterwards does nothing more. It works like Button A with that camera
selected:

- not recording → **start recording** (in photo mode: take a photo)
- recording → **stop recording**
- asleep → **wake up and take a snapshot**
- paired but not connected → **connect** (same as **Connect** on Button A)

The long press works with every selection, including combinations such as
**Slot 1 + 2 + 3**. It is ignored on an empty slot (it does not open
pairing), while a camera is still connecting, and on a camera that is not
responding (use **Connect** on Button A or in the Options Menu for that). It
also does nothing while the remote is still searching for cameras after
startup.

While a camera connects, the other buttons react only after the connection
attempt has finished (same as with **Connect** on Button A).

### Busy Indicator (Waveshare)

On the Waveshare ESP32-S3-LCD-1.9 there is no message line. Instead, a
**cyan circle with two arrows** shows that the remote has reacted and an
action on a camera is still running:

- On the **Main Screen** it appears in the lower half of that camera's
  block (over the SD card and battery rows). The camera name, status icon
  and recording time stay visible.
- In the **Options menu** it appears in the bottom-left corner for the
  camera the menu belongs to.

It appears as soon as you trigger the action. For a long press, that is
when 1 s is reached, while you are still holding the button. It is shown
for:

| Action | Disappears when |
|--------|-----------------|
| Connect (button, at startup, after pairing) | the camera is connected |
| Start recording / take a photo | the camera records / the photo is taken |
| Stop recording | the camera has stopped |
| Sleep | the camera sleeps |
| Wake up, snapshot on a sleeping camera | the camera is awake |
| Disconnect | the camera is disconnected |
| Unpair | the camera is unpaired |

It also disappears if the action fails, the camera disconnects or stops
responding, and after **10 seconds** at the latest. In multi-camera mode
every affected camera shows its own indicator, and each one disappears on
its own. Highlight tags and Quick Switch show no indicator. An action
started in the Options menu stays visible after you go back to the Main
Screen.

The M5Stack shows messages in its message line instead and has no busy
indicator.


# 1. Starting the Remote

When powering on, the device shows a splash screen and then loads the **Main Screen**.  
Startup behavior differs depending on whether cameras were previously paired.


## 1.1 If No Cameras Are Paired

If all three camera slots are unpaired, the Main Screen shows empty camera blocks.

### Pairing Cameras

1. Press **Button B** to select a camera slot  
   (factory cycle: Slot 1 → Slot 2 → Slot 3 → Slot 1 + 2 + 3 → …)  
2. Stop on a slot showing **Not paired**.  
3. Press **Button A** or **Button C** to open the **Pairing Menu**.  
4. The remote scans for nearby DJI cameras.  
5. Use **Button B** to select a camera from the list.  
6. Press **Button A** to pair it to the selected slot.  
7. Press **Button C** at any time to cancel the scan and go back.

While no camera has been found yet, the screen shows **Scanning...**. On the
Waveshare (320×170) display the list holds up to 8 cameras in two columns;
long names end with "…".

### Camera Slots Remain Persistent

Each paired camera stays assigned to its slot permanently.  
Reboots do not remove pairings.


## 1.2 If Cameras Are Already Paired

When paired cameras exist, the device performs an **automatic boot scan**.

### Automatic Reconnection

- The remote searches for all paired cameras at once.
- As cameras are found, they appear with a **green “found” icon**.
- The scan ends as soon as all paired cameras are found. If a camera is
  missing (for example switched off), the scan ends about **10 seconds after
  the first camera was found**, and at the latest after 30 seconds.
- After the scan ends, connection begins automatically. A camera that was
  not found is shown as paired, not connected; connect it later with
  **Button A**.
- If you restart the remote while the cameras were connected (for example
  switch it off and on again at once), the cameras need up to about
  5 seconds before they can be found again. They first have to notice that
  the old connection is gone.

### Stopping the Boot Scan

During the active scan:

- Only **Button B** and **Button C** are active.
- **Button C** displays **Stop**, with a scanning icon. Press it to
  immediately stop scanning and start connecting to already discovered
  cameras.
- **Button B** displays **Slots**. Press it to open the slot cycle
  configuration (see "Slots" in 2.4). The scan continues in the background;
  when you go back, the Main Screen shows the current state. If the scan ends
  while you are still in the configuration, the cameras connect
  automatically; the buttons may react with a short delay during that time.

### Manual Reconnect After Boot

If a paired camera did not connect:

1. Select its slot using **Button B**.
2. Press **Button A** (“Connect”).

The camera status icon changes to **connecting** immediately, and a focused
reconnect scan for that specific camera begins.

### If a Camera Loses Its Connection

If a camera drops its connection during use (for example, it is switched off
or goes out of range):

- Its status icon switches to the **paired, not connected** icon. There is no
  message and no sound.
- The other cameras keep working normally: status, GPS and recording control
  are not affected.
- The remote does **not** reconnect on its own. To reconnect, select that
  camera with **Button B** and press **Button A** ("Connect").
- Short radio dropouts (a few seconds, for example when your body or the
  bike shields the camera) do not close the connection. A camera that stays
  out of range is shown as disconnected after about **5 seconds**.
- In multi-camera mode, disconnected cameras are skipped.
- A camera that was asleep when it lost the connection is shown as awake
  afterwards. Its real state is shown again as soon as it is reconnected.

### Messages After a Button Press

Camera commands are sent in the background, so the remote never waits for a
camera:

- Result messages such as "Recording Started" or "Photo Taken" can appear a
  moment after the press.
- If you press a button again before the camera has answered the previous
  command, **"Busy"** is shown and the second press is ignored. Just press
  again a moment later.
- A camera that does not answer no longer freezes the remote. Its command
  fails after a few seconds ("Start Failed", "Stop Failed"), while the other
  cameras and all buttons keep working.

### If a Camera Stops Responding

Sometimes a camera stays connected but stops sending its status. The remote
watches every connected, awake camera for this:

- After about **6 seconds** without status, the camera's status icon changes
  to a **yellow warning triangle**. Commands and GPS are no longer sent to it.
- If the camera starts sending again, the normal icon returns on its own.
- After about **15 seconds** without status, the remote closes the connection.
  The camera is then shown as **paired, not connected**.
- To recover the camera right away, select it with **Button B** and press
  **Button A** ("Connect"), or use **Settings → Connect**. The remote closes
  the old connection first and then reconnects. If the camera is out of
  range, closing the old connection can take up to about 6 seconds; the
  connecting icon is shown meanwhile.
- In multi-camera mode, a camera that is not responding is skipped.
- In Settings, **Sleep/Wakeup** and **Mode Switch** show "Not responding" for
  such a camera. **Unpair** still works.
- Sleeping cameras are not watched; a sleeping camera never shows the
  triangle.

### Low SD Card and Battery Warnings

Each camera block shows the free space on the SD card and the battery
level at the bottom. When a value gets low, the whole row is colored so you
can see it at a glance:

| Row | Yellow row (black text) | Red row (white text) |
|-----|-------------------------|----------------------|
| SD card | 16 GB free or less | 6 GB free or less |
| Battery | 25 % or less | 10 % or less |

The two rows are independent: for example, the SD card row can be yellow
while the battery row stays normal. The rows (and their colors) are shown
for connected cameras, including sleeping ones.

Firmware builds can instead use the older style, where only the text and
icon turn yellow or red (build option, see the build documentation).


# 2. Single-Camera Mode

Single-camera mode is active whenever the selected cycle entry contains
exactly one camera slot (Slot 1, 2 or 3).


## 2.1 Selecting a Camera

Press **Button B** to move to the next entry of the **slot cycle**. The
factory cycle is:

> Slot 1 → Slot 2 → Slot 3 → Slot 1 + 2 + 3 → …

The cycle has four entries and can be changed under **Slots** (see 2.4):
from the Options Menu, or with **Button B** while the remote searches for
its cameras after switching on. Each entry is a single slot, a combination of slots
(e.g. **Slot 1 + 3**) or **Empty**. Empty entries are skipped.

Every selected camera has a red highlight bar; with a combination, all
cameras of the entry are highlighted.


## 2.2 Recording Control

Press **Button A**:

- If not recording → **Start recording**
- If recording → **Stop recording**

To control **another** camera without changing the selection, hold its
button for 1 s (A = Camera 1, B = Camera 2, C = Camera 3; see
"Long Press on the Main Screen").


## 2.3 Adding Highlight Tags

Available only when:

- The camera is connected  
- It is awake  
- It is currently recording  
- It supports highlight tags (Action 4, Action 5 Pro, Action 6)

Press **Button C** to insert a **Highlight Tag**.

Unsupported models (e.g., Osmo 360) do not offer this feature.


## 2.4 Options Menu

Press **Button C** when the selected camera is **not recording**.

The Options Menu includes:

- Mode Switch  
- Sleep / Wake  
- Connect / Disconnect  
- Pair / Unpair  
- Slots (always last)

Use **Button B** to navigate, **Button A** to select.  
Press **Button C** to go back to the Main Screen.

On the Waveshare (320x170) the five entries are shown in two columns.

### Mode Switch

Shows the camera's current mode.

- **Button A** — switch to the next camera mode (Quick Switch)
- **Button B** — no function
- **Button C** — go back to the Main Screen

### Slots (Configuring the Slot Cycle)

Defines which selections **Button B** cycles through on the Main Screen.
The setting is global: it is the same no matter from which camera the
Options Menu was opened, and it is available in every connection state.

There are two ways to open it:

- **Options Menu → Slots** (needs an entry with exactly one camera, because
  only such an entry opens the Options Menu)
- **Button B "Slots"** during the boot scan after switching on (see
  "Stopping the Boot Scan" in 1.2). This works whenever at least one camera
  is paired, even if no entry has a single camera.

**Screen 1 – Cycle entries** lists the four entries in cycle order by name,
e.g. **Slot 1**, **Slot 1 + 2**, **Slot 1 + 3**, **Empty**.

- **Button A** (Select) — edit the highlighted entry (Screen 2)
- **Button B** (Next) — highlight the next entry
- **Button C** (Back) — go back to the Main Screen and save the cycle

**Screen 2 – Slots of one entry** lists Slot 1, Slot 2 and Slot 3. A green
check mark at the right edge marks every slot that belongs to the entry.

- **Button A** (Select) — add or remove the highlighted slot
- **Button B** (Next) — highlight the next slot
- **Button C** (Back) — go back to Screen 1

Rule: at least one entry must contain a camera. Removing the last camera of
the last non-empty entry is refused: the row flashes red for about one
second and nothing changes.

A cycle of combinations only (e.g. **Slot 1 + 2** → **Slot 1 + 2 + 3**) is
allowed. The Options Menu (Mode Switch, Sleep/Wake, Connect, Pair/Unpair)
then cannot be opened; to get it back, add an entry with a single camera via
**Button B "Slots"** during the next boot scan.

The cycle is saved when you leave Screen 1 with **Back** and survives a
restart. If the entry that was selected is now Empty, the selection moves to
the next non-empty entry. To restore the factory cycle, edit the entries back
to Slot 1 → Slot 2 → Slot 3 → Slot 1 + 2 + 3.


## 2.5 Snapshot While Sleeping

If the selected camera is asleep, **Button A** shows the record icon with the
label **"Snapsht"** (short for Snapshot):

1. Press **Button A**.  
2. The remote sends a **Wake** command.  
3. As soon as the camera wakes, the remote sends a **Snapshot** command.  
4. Many cameras return to sleep afterward.

The remote handles wake timing and synchronization automatically.


# 3. Multi-Camera Mode

Active when the selected cycle entry contains two or more cameras, e.g.
**Slot 1 + 2 + 3** (formerly "All Cameras") or **Slot 1 + 3**. Everything in
this section applies **only to the cameras of the selected entry**; the
other cameras are not affected. "All cameras" below means "all cameras of
the entry". The Options Menu is not available in this mode.


## 3.1 Start/Stop Recording for All Cameras of the Entry

Press **Button A**:

- If *any* connected camera is recording → **Stop all**
- Otherwise → **Start all**

Sleeping cameras are awakened automatically and sequentially when needed.

To start or stop **one** camera while a combination stays selected, hold its
button for 1 s: **A** = Camera 1, **B** = Camera 2, **C** = Camera 3. The
other cameras are not affected.


## 3.2 Sleep & Wake All Cameras of the Entry

Press **Button C**.

Priority logic:

1. If any cameras are recording → **Highlight Tags**  
2. Else if cameras are awake → **Sleep All**  
3. Else if cameras are sleeping → **Wake All**  
4. Else → No action  


## 3.3 Highlight Tags for All Cameras of the Entry

If multiple cameras are recording and support highlight tagging:

- Press **Button C**
- A highlight tag is sent to each eligible camera
- The remote shows how many received the tag


## 3.4 Sequential Wake Behavior

DJI cameras require **individual wake sequences**.

The remote automatically:

1. Wakes one camera  
2. Waits for confirmation  
3. Sends the corresponding action (record/s snapshot/highlight)  
4. Continues with the next camera  

This ensures high reliability across multiple models.


## 3.5 Snapshot in Multi-Camera Mode

If all connected cameras are asleep, **Button A** shows the label **"Snapsht"**
(short for Snapshot). Pressing it:

- Each sleeping camera is woken one at a time  
- After waking, it receives a **Snapshot** command  
- The process continues until all sleeping cameras were handled


# 4. GPS Features

The remote reads GPS data from the external GPS module and displays it in the GPS area.


## 4.1 GPS Information Display

The GPS icon is a **map pin** at the bottom of the Main Screen.

### No Fix

- Red GPS icon  
- No coordinates shown  

### Valid Fix

- Yellow GPS icon (green on the Waveshare)  
- Latitude & longitude shown with two decimals (M5Stack only; the Waveshare
  shows the icon only)  

Updates occur even when cameras are asleep.


## 4.2 GPS Data Sent to Cameras

For each connected *and awake* camera, the remote sends:

- Latitude  
- Longitude  
- Altitude  
- Speed  
- Date and time (GPS UTC time; the remote has no clock or time-zone setting of its own)  
- Fix state  

Transmission is continuous (≈10 Hz). GPS data is only sent while the GPS
module reports a fix **and** a current date and time; without them, the
cameras get no GPS data rather than a wrong time.

> **Note on the recorded GPS time:** The cameras store the GPS time in the
> clip metadata (`GPSDateTime`, e.g. visible with `exiftool -ee`) **one hour
> earlier** than UTC. This is how the camera firmware handles the received
> time (tested on Osmo Action 5 Pro, Osmo Action 6 and Osmo 360). The date
> and time shown in DJI Mimo (file name, dashboard) come from the camera's
> own clock and are not affected.

If a camera sleeps, GPS transmission pauses automatically and resumes when it wakes.


# 5. Fault Diagnostics

The remote keeps a small record of connection problems and restarts, even
without a computer attached. After a tour you can read it over USB to see
what happened, when, and with which camera:

- restarts of the remote (with the reason, e.g. power-on or crash)
- cameras that lost their connection unexpectedly
- cameras that stopped responding, recovered, or were disconnected by the
  remote
- connections that dropped while connecting

If the GPS had a fix, each entry also has the date and time.

To read the record, connect the remote by USB and run
`tools/diag/dji_diag read -p <port>` (see
[`tools/diag/README.md`](../tools/diag/README.md)). Reading restarts the
remote. Your pairings are not affected, and updating the firmware keeps the
record.
