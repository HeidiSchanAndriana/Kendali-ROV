# Menambahkan Algoritma Kendali ke `rov_ws`

## Konsep
- Tulis algoritma Anda sebagai node ROS 2 yang menerbitkan `geometry_msgs/Twist` ke topik **`/control/cmd`**.
  - `linear.x` = surge (maju/mundur) dalam rentang **[-1, 1]**
  - `linear.y` = sway (kiri/kanan) dalam rentang **[-1, 1]**
  - `linear.z` = heave (naik/turun), **positif = naik**, rentang **[-1, 1]**
  - `angular.z` = yaw (kiri/kanan) dalam rentang **[-1, 1]**
- Paket `rov_ctrl_if` berperan sebagai **mux/adapter**, yang mengubah Twist menjadi `mavros_msgs/ManualControl` (0..1000) dengan netral 500.

## Langkah Cepat
1. Tambahkan paket algoritma Anda (mis. `src/rov_my_alg/`).
2. Buat node yang publish `Twist` ke `/control/cmd` pada 10–50 Hz.
3. Jalankan stack:
   ```bash
   # start SITL + MAVProxy + MAVROS
   ./scripts/rov_start.sh

   # jalankan interface mux + algoritma Anda
   ros2 run rov_ctrl_if control_mux &
   ros2 run rov_ctrl_alg_template alg_runner   # atau node Anda sendiri
   ```

## Template yang Disediakan
- `rov_ctrl_alg_template/algorithms/base.py` — base class sederhana.
- `rov_ctrl_alg_template/nodes/alg_runner.py` — runner yang memuat kelas algoritma via string `modul:kelas`.
- `rov_ctrl_alg_template/algorithms/pid_hold_depth.py` — contoh PID kedalaman minimal.
- `rov_ctrl_if/nodes/control_mux.py` — adapter Twist → ManualControl dengan deadman.

## Parameterisasi
- Atur parameter algoritma melalui YAML (`config/example_pid.yaml`) atau `--ros-args -p params:="{\"kp\":1.0}"`.

## Best Practice
- Batasi output [-1,1] dan biarkan `control_mux` yang melakukan scaling.
- Gunakan `deadman_timeout` agar thrust kembali netral bila node Anda berhenti mengirim perintah.
- Pisahkan logic control vs device/transport (MAVROS) agar mudah dipindah ke hardware lain.
