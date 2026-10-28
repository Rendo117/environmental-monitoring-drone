# Firebase set-up (free "Spark" plan)

The 2022 prototype used a Realtime Database that anyone who knew the URL could read
and write. That means a stranger could have sent flight commands. This folder holds
the configuration that closes that hole.

## 1. Create the project
1. <https://console.firebase.google.com> → **Add project** (Analytics not needed).
2. **Build → Realtime Database → Create database**, location `europe-west1`, start in *locked mode*.
3. **Build → Authentication → Get started → Email/Password → Enable**.
4. **Project settings → General → Web API key**: copy it (it identifies the project; it is not a secret, the rules below are the protection).

## 2. Create two users (Authentication → Users → Add user)
| user | used by | role |
|---|---|---|
| `operator@…` | Java ground station | may write commands, read telemetry |
| `drone1@…`   | Raspberry Pi bridge | may read commands, write telemetry |

Copy each user's **UID**.

## 3. Grant the roles (Realtime Database → Data → import JSON)
```json
{
  "operators": { "<OPERATOR_UID>": true },
  "aircraft":  { "drone1": { "<DRONE_UID>": true } }
}
```

## 4. Publish the rules
Paste [`database.rules.json`](database.rules.json) in **Realtime Database → Rules → Publish**
(or `firebase deploy --only database` with the Firebase CLI).

The rules enforce:
* nobody unauthenticated can read or write anything;
* only operators can write `/drones/<id>/command`;
* only that drone's account can write `/drones/<id>/telemetry`;
* every stick value must be a number in 0…100, extra fields are rejected.

## Data model
```
/drones/drone1/command            {roll, pitch, yaw, throttle (0..100), seq, mode, sentAt}
/drones/drone1/telemetry/latest   {status, failsafe, roll_deg, pitch_deg, yaw_rate_dps,
                                   gas_adc, gas_lpg_ppm, gas_co_ppm, gas_smoke_ppm,
                                   geiger_cps, geiger_counts_60s, radiation_cpm,
                                   radiation_usv_h, radiation_sigma_usv_h, battery_v, time, …}
/drones/drone1/telemetry/history/<pushId>   same, 1 Hz log
```
Free-plan limits (1 GB stored, 10 GB/month downloaded) are plenty: one telemetry record is
about 0.5 kB, so 1 Hz logging is roughly 43 MB per 24 h of flight. Delete old history from time to time.
