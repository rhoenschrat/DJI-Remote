# DJI-Remote – User Manual

This manual explains how to operate the DJI-Remote device, covering startup behavior, pairing, single-camera control, multi-camera coordination, and GPS functionality.


# 1. Starting the Remote

When powering on, the device shows a boot logo and then loads the **Main Screen**.  
Startup behavior differs depending on whether cameras were previously paired.


## 1.1 If No Cameras Are Paired

If all three camera slots are unpaired, the Main Screen shows empty camera blocks.

### Pairing Cameras

1. Press **Button B** to select a camera slot  
   (Camera 1 → Camera 2 → Camera 3 → All Cameras → …)  
2. Stop on a slot showing **Not paired**.  
3. Press **Button A** or **Button C** to open the **Pairing Menu**.  
4. The remote scans for nearby DJI cameras.  
5. Use **Button B** to select a camera from the list.  
6. Press **Button A** to pair it to the selected slot.

### Camera Slots Remain Persistent

Each paired camera stays assigned to its slot permanently.  
Reboots do not remove pairings.


## 1.2 If Cameras Are Already Paired

When paired cameras exist, the device performs an **automatic boot scan**.

### Automatic Reconnection

- The remote searches for all paired cameras at once.
- As cameras are found, they appear with a **green “found” icon**.
- After the scan ends or all cameras are found, connection begins automatically.

### Stopping the Boot Scan

During the active scan:

- Only **Button C** is active.
- It displays **Stop**, with a scanning icon.
- Press **Button C** to immediately stop scanning and start connecting to already discovered cameras.

### Manual Reconnect After Boot

If a paired camera did not connect:

1. Select its slot using **Button B**.
2. Press **Button A** (“Connect”).

A focused reconnect scan for that specific camera begins.


# 2. Single-Camera Mode

Single-camera mode is active whenever one camera (Camera 1, 2, or 3) is selected.


## 2.1 Selecting a Camera

Press **Button B** to cycle:

> Camera 1 → Camera 2 → Camera 3 → All Cameras → …

The selected camera has a red highlight bar.


## 2.2 Recording Control

Press **Button A**:

- If not recording → **Start recording**
- If recording → **Stop recording**


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

- Back  
- Mode Switch  
- Sleep / Wake  
- Connect / Disconnect  
- Unpair  

Use **Button B** to navigate, **Button A** to select.


## 2.5 Snapshot While Sleeping

If the selected camera is asleep:

1. Press **Button A**.  
2. The remote sends a **Wake** command.  
3. As soon as the camera wakes, the remote sends a **Snapshot** command.  
4. Many cameras return to sleep afterward.

The remote handles wake timing and synchronization automatically.


# 3. Multi-Camera Mode

Active when the selection is set to **All Cameras**.


## 3.1 Start/Stop Recording for All Cameras

Press **Button A**:

- If *any* connected camera is recording → **Stop all**
- Otherwise → **Start all**

Sleeping cameras are awakened automatically and sequentially when needed.


## 3.2 Sleep & Wake All Cameras

Press **Button C**.

Priority logic:

1. If any cameras are recording → **Highlight Tags**  
2. Else if cameras are awake → **Sleep All**  
3. Else if cameras are sleeping → **Wake All**  
4. Else → No action  


## 3.3 Highlight Tags for All Cameras

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

If cameras are asleep and you press **Button A**:

- Each sleeping camera is woken one at a time  
- After waking, it receives a **Snapshot** command  
- The process continues until all sleeping cameras were handled


# 4. GPS Features

The remote reads GPS data from the external GPS module and displays it in the GPS area.


## 4.1 GPS Information Display

### No Fix

- Red GPS icon  
- No coordinates shown  

### Valid Fix

- Yellow GPS icon  
- Latitude & longitude shown with two decimals  

Updates occur even when cameras are asleep.


## 4.2 GPS Data Sent to Cameras

For each connected *and awake* camera, the remote sends:

- Latitude  
- Longitude  
- Altitude  
- Speed  
- Timestamp  
- Fix state  

Transmission is continuous (≈10 Hz).

If a camera sleeps, GPS transmission pauses automatically and resumes when it wakes.
