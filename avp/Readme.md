# Apple Vision Pro Communication Setup (Vuer / OpenTeleVision)

No Mac, no Xcode, no Apple Developer account needed.
The PC runs an HTTPS+WSS Vuer server; the AVP joins from Safari into a WebXR
immersive session, and the PC reads 4×4 head and hand poses from Vuer's
`HAND_MOVE` / `CAMERA_MOVE` events.

```
[Apple Vision Pro]                                     [PC (Linux)]
 Safari (WebXR session)  ◄── HTTPS:8012 + WSS:8012 ──► Vuer server
   └─ Don the headset, enter immersive                  └── OpenTeleVision
       └─ head/hand pose events                              ├── .head_matrix    (4,4)
                                                             ├── .left_hand      (4,4)
                                                             ├── .right_hand     (4,4)
                                                             ├── .left_landmarks (25,3)
                                                             └── .right_landmarks(25,3)
```

Reference implementation: [../egox/data_collect/tele_vision.py](../egox/data_collect/tele_vision.py).

---

## 1. PC environment

```bash
conda create -n avp python=3.10 -y
conda activate avp
pip install vuer==0.0.31rc7 aiohttp==3.9.5 aiohttp_cors==0.7.0 numpy
```

Pin `vuer` to the version listed in egox's `requirements.txt`; newer releases
changed the schema and the event payloads will not line up.

> **Python 3.8 compatibility (this machine's `aloha` env)**
> A default install pulls in `params_proto-3.x`, which uses PEP 585 syntax
> (`tuple[Any, bool]`) and crashes at import on 3.8.
> Fix: `pip install 'params_proto<3.0'` (verified: 2.13.2 imports fine on
> Python 3.8 with vuer 0.0.31rc7).

---

## 2. Generate a self-signed certificate (mkcert)

WebXR in Safari only enables for **HTTPS origins**, so the PC must serve a
trusted TLS certificate. The simplest way is a self-signed CA plus a server
certificate for the host's IP, both produced by
[mkcert](https://github.com/FiloSottile/mkcert).

### 2.1 Install mkcert

```bash
# Ubuntu / Debian
sudo apt install -y libnss3-tools
curl -L https://github.com/FiloSottile/mkcert/releases/latest/download/mkcert-v1.4.4-linux-amd64 \
     -o /usr/local/bin/mkcert
sudo chmod +x /usr/local/bin/mkcert
```

### 2.2 Generate the root CA and server certificate

```bash
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/avp
mkcert -install                                # add root CA to system trust store
mkcert -cert-file cert.pem -key-file key.pem \
       <PC_LAN_IP> localhost 127.0.0.1         # bake the LAN IP into the cert
```

This creates `cert.pem` and `key.pem` in the current directory; Vuer will load
them at startup. The mkcert root CA file lives at `$(mkcert -CAROOT)/rootCA.pem`,
**which the next step pushes onto the AVP**.

> Note: `<PC_LAN_IP>` must be the address the AVP can actually reach the PC on
> (same Wi-Fi segment). The certificate is bound to the listed IPs; if the PC
> changes networks, re-issue.

---

## 3. Install the mkcert root CA on AVP (one-off)

After this step Safari on the AVP will trust your PC's HTTPS. The flow mirrors iOS.

1. PC: serve the root CA over a temporary HTTP server.
   ```bash
   cd "$(mkcert -CAROOT)"
   python3 -m http.server 8000
   ```
2. AVP Safari: open `http://<PC_LAN_IP>:8000/rootCA.pem` → "Download
   Configuration Profile" prompt appears.
3. AVP `Settings → General → VPN & Device Management → Downloaded Profile →
   Install` (you may be asked for the AVP passcode twice).
4. AVP `Settings → General → About → Certificate Trust Settings` → find the
   newly installed mkcert CA → toggle "Enable Full Trust for Root Certificates"
   to on.

> visionOS menu wording may differ slightly. Look for "Profile" and
> "Certificate Trust".

---

## 4. Enable WebXR on AVP

Some visionOS versions ship with WebXR off by default. Turn it on manually:

`Settings → Apps → Safari → Advanced → Feature Flags` →
- `WebXR Device API` → On
- `WebXR Hand Input Module` → On
- `WebXR Augmented Reality Module` → On (optional)

After flipping the flags, **fully quit and relaunch Safari**.

---

## 5. Start the Vuer server and enter WebXR on AVP

PC: run the minimal test script (provided in the next section):

```bash
cd /home/agilex/cobot_magic/aloha-devel/Piper-AVP-Teleop/avp
python avp_min_test.py
```

It will listen on `0.0.0.0:8012` over HTTPS+WSS.

AVP Safari: open

```
https://<PC_LAN_IP>:8012?ws=wss://<PC_LAN_IP>:8012
```

Once the page loads, the WebXR standard UI shows an **"Enter VR"** button
somewhere on screen — tap it to start an immersive session. Once you are
inside, the PC log will start printing 4×4 head and hand pose matrices.

---

## 6. Reading pose data (API)

`OpenTeleVision` stores Vuer events in multiprocessing shared arrays. The main
process reads them through properties:

```python
vr.head_matrix        # (4,4) headset pose, AVP world frame
vr.left_hand          # (4,4) left wrist
vr.right_hand         # (4,4) right wrist
vr.left_landmarks     # (25,3) left hand 25 keypoints (joints)
vr.right_landmarks    # (25,3) right hand 25 keypoints
vr.aspect             # AVP display aspect ratio (scalar)
```

Property definitions: [tele_vision.py:180-202](../egox/data_collect/tele_vision.py#L180-L202).

### 6.1 Coordinate frame
- AVP / WebXR world frame: **y up**, origin = where the headset locks tracking
  on entering the immersive session.
- Different from the IsaacGym / robot convention (z up, x forward) — needs an
  axis remap. egox's remap lives in
  [egox_data_collect.py:1454-1472](../egox/data_collect/egox_data_collect.py#L1454-L1472).
- Units: meters / radians.

### 6.2 "Pose relative to initial"
Vuer reports absolute poses. Lock a frame the moment the operator settles into
the starting posture:
```python
T0 = vr.head_matrix.copy()
# every subsequent frame:
T_rel = np.linalg.inv(T0) @ vr.head_matrix
```
egox does this more carefully: it takes only the head's yaw at reset, rotates
the world frame about z so "operator forward = +x", and uses the reset-frame
head position as the origin for subsequent deltas. See
[egox_data_collect.py:1579-1637](../egox/data_collect/egox_data_collect.py#L1579-L1637).

---

## 7. Verification flow

1. PC: `python avp_min_test.py`; log shows
   `Vuer server running on https://0.0.0.0:8012`.
2. AVP Safari: open `https://<PC_LAN_IP>:8012?ws=wss://<PC_LAN_IP>:8012` —
   **no certificate warning** (if you see one, step 3 didn't take).
3. Tap "Enter VR" to enter immersive.
4. Move your head and hands — the `head_matrix` / `left_hand` / `right_hand`
   matrices printed on the PC terminal should track in real time.

---

## 8. Common gotchas

- **Safari keeps flagging the cert as untrusted / refuses to load**: the
  `mkcert -CAROOT` root CA isn't installed on the AVP, or it is installed but
  not toggled on under Certificate Trust Settings.
- **Page loads but no "Enter VR" button**: WebXR feature flags aren't enabled
  (section 4), or Safari wasn't restarted.
- **Inside VR but `head_matrix` is all zeros / identity**: Safari WebXR isn't
  emitting events. Most often you didn't actually enter the immersive session
  (just opening the page isn't enough). Tap Enter VR.
- **`left_hand` sometimes jumps to a strange pose**: WebXR hand tracking falls
  back to the previous frame or returns garbage when the hand leaves the FOV.
  egox's `safe_mat_update` reuses the previous frame in that case. See
  [egox_data_collect.py:1421-1444](../egox/data_collect/egox_data_collect.py#L1421-L1444).
- **PC's IP changed and AVP can't connect**: certificate is bound to the IP you
  signed it with — re-run `mkcert -cert-file ... <new IP> ...` and restart the
  server.
- **High latency / stutter**: prefer 5 GHz Wi-Fi, keep PC and AVP on the same
  router/segment, kill any heavy traffic in the area.
- **Can't see the real world**: Vuer runs in VR mode, not pass-through. You can
  push a camera feed into the Vuer scene background — see the `ImageBackground`
  usage in [tele_vision.py:88-171](../egox/data_collect/tele_vision.py#L88-L171)
  — but you won't see your actual room.
