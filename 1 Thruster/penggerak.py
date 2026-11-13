import argparse
import math
import time
from pymavlink import mavutil

# ===================== PENGATURAN =====================
BAUD = 115200          # kecepatan serial (tidak berpengaruh bila lewat kabel USB)
GAS_MAKS = 0.3         # batas gas, 0.0 sampai 1.0. Naikkan pelan-pelan.
DETIK_PER_LANGKAH = 3  # lama tiap langkah dalam urutan tes
GAIN_BELOK = 0.8       # seberapa jauh (radian) arah tujuan dibelokkan saat yaw = 1.0
KECEPATAN_KIRIM = 20   # perintah dikirim 20 kali per detik
MODE_GUIDED = 15       # nomor mode GUIDED di ArduRover
PWM_TENGAH = 1500      # thruster diam
PWM_MIN = 1100         # thruster mundur penuh
PWM_MAX = 1900         # thruster maju penuh

URUTAN_TES = [
    ("Maju lurus", 1.0, 0.0),
    ("Berhenti", 0.0, 0.0),
    ("Mundur lurus", -1.0, 0.0),
    ("Berhenti", 0.0, 0.0),
    ("Maju belok kanan", 1.0, 0.5),
    ("Maju belok kiri", 1.0, -0.5),
    ("Berhenti", 0.0, 0.0),
]

# ===================== DIFFERENTIAL DRIVE =====================
class DifferentialDrive:
    def campur(self, surge, yaw):
        surge = max(-1.0, min(1.0, surge))
        yaw = max(-1.0, min(1.0, yaw))
        selisih = PWM_MAX - PWM_TENGAH
        kiri = PWM_TENGAH + (surge + yaw) * selisih
        kanan = PWM_TENGAH + (surge - yaw) * selisih
        kiri = max(PWM_MIN, min(PWM_MAX, kiri))
        kanan = max(PWM_MIN, min(PWM_MAX, kanan))
        return int(kiri + 0.5), int(kanan + 0.5)

# ===================== PIXHAWK =====================
class Pixhawk:
    def __init__(self, port, baud):
        print(f"Menghubungkan ke Pixhawk di {port} ...")
        try:
            self.master = mavutil.mavlink_connection(port, baud=baud)
        except Exception as galat:
            raise SystemExit(f"Gagal membuka {port}: {galat}")
        if self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=10) is None:
            raise SystemExit("Tidak ada jawaban dari Pixhawk. Cek kabel USB dan nama port, "
                             "lalu pastikan QGroundControl sudah ditutup.")
        print(f"Pixhawk terhubung (sistem {self.master.target_system}).")
        self.yaw = None  # arah kapal sekarang (radian), dari pesan ATTITUDE
        self.armed = False
        self.mode = None
        self._heartbeat_terakhir = 0.0
        # Minta Pixhawk mengirim ATTITUDE 20 kali per detik.
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50000, 0, 0, 0, 0, 0)

    def perbarui(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break
            jenis = msg.get_type()
            if jenis == "ATTITUDE":
                self.yaw = msg.yaw
            elif jenis == "HEARTBEAT" and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = msg.custom_mode
            elif jenis == "STATUSTEXT":
                print(f"\n[Pixhawk] {msg.text}")

    def kirim_heartbeat(self):
        sekarang = time.time()
        if sekarang - self._heartbeat_terakhir >= 1.0:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._heartbeat_terakhir = sekarang

    def tunggu(self, kondisi, detik):
        batas = time.time() + detik
        while time.time() < batas:
            self.perbarui()
            self.kirim_heartbeat()
            if kondisi():
                return True
            time.sleep(0.05)
        return False

    def atur_mode(self, nomor_mode):
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, nomor_mode)

    def arm(self):
        self._arm_disarm(1)

    def disarm(self):
        self._arm_disarm(0)

    def _arm_disarm(self, nilai):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, nilai, 0, 0, 0, 0, 0, 0)

    def gerak(self, surge, yaw):
        if self.yaw is None:
            return  # arah kapal belum diketahui, jangan kirim apa-apa
        surge = max(-1.0, min(1.0, surge))
        yaw = max(-1.0, min(1.0, yaw))
        arah_tujuan = self.yaw + yaw * GAIN_BELOK
        q = [math.cos(arah_tujuan / 2), 0.0, 0.0, math.sin(arah_tujuan / 2)]
        self.master.mav.set_attitude_target_send(
            0, self.master.target_system, self.master.target_component,
            0b00000111,  # abaikan kecepatan putar roll, pitch, yaw: pakai arah dan gas
            q, 0, 0, 0, surge)

    def berhenti(self):
        for _ in range(10):
            self.gerak(0.0, 0.0)
            time.sleep(0.05)
        self.disarm()

# ===================== PROGRAM UTAMA =====================
def tampilkan_campuran():
    drive = DifferentialDrive()
    print(f"{'Langkah':<18} {'surge':>6} {'yaw':>6} {'kiri':>6} {'kanan':>6}")
    for nama, surge, yaw in URUTAN_TES:
        kiri, kanan = drive.campur(surge, yaw)
        print(f"{nama:<18} {surge:>+6.1f} {yaw:>+6.1f} {kiri:>6} {kanan:>6}")


def jalankan_tes(port, baud, gas, detik):
    drive = DifferentialDrive()
    print("PERINGATAN: thruster akan berputar. Kapal harus diangkat dari air atau propeller dilepas.")
    if input("Sudah aman? Ketik y lalu Enter untuk lanjut: ").strip().lower() != "y":
        print("Dibatalkan.")
        return
    pixhawk = Pixhawk(port, baud)
    try:
        if not pixhawk.tunggu(lambda: pixhawk.yaw is not None, 5):
            raise SystemExit("Arah kapal (ATTITUDE) belum diterima dari Pixhawk.")
        print("Pindah ke mode GUIDED ...")
        pixhawk.atur_mode(MODE_GUIDED)
        if not pixhawk.tunggu(lambda: pixhawk.mode == MODE_GUIDED, 5):
            raise SystemExit("Mode GUIDED ditolak Pixhawk. Baca pesan [Pixhawk] di atas.")
        print("ARM ...")
        pixhawk.arm()
        if not pixhawk.tunggu(lambda: pixhawk.armed, 10):
            raise SystemExit("ARM ditolak Pixhawk. Baca pesan [Pixhawk] di atas.")
        for nama, surge, yaw in URUTAN_TES:
            akhir = time.time() + detik
            while time.time() < akhir:
                pixhawk.perbarui()
                pixhawk.kirim_heartbeat()
                pixhawk.gerak(surge * gas, yaw)
                # Pratinjau kiri/kanan: surge dan yaw sama-sama dikali gas supaya sama kecilnya.
                kiri, kanan = drive.campur(surge * gas, yaw * gas)
                arah = math.degrees(pixhawk.yaw) % 360
                print(f"\r{nama:<18} surge {surge * gas:+.2f} yaw {yaw:+.2f} | "
                      f"kiri {kiri} kanan {kanan} | arah kapal {arah:5.1f}   ",
                      end="", flush=True)
                time.sleep(1.0 / KECEPATAN_KIRIM)
            print()
    finally:
        print("Berhenti: thruster 0 lalu DISARM.")
        pixhawk.berhenti()


def main():
    parser = argparse.ArgumentParser(
        description="Menggerakkan thruster lewat Pixhawk dengan program Python.")
    parser.add_argument("--hitung", action="store_true",
                        help="hitung campuran kiri dan kanan tanpa Pixhawk")
    parser.add_argument("--port", help="port Pixhawk, contoh: COM5 atau /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=BAUD, help="baud rate (bawaan 115200)")
    parser.add_argument("--gas", type=float, default=GAS_MAKS,
                        help="batas gas, lebih dari 0 sampai 1.0 (bawaan 0.3)")
    parser.add_argument("--detik", type=float, default=DETIK_PER_LANGKAH,
                        help="lama tiap langkah dalam detik (bawaan 3)")
    args = parser.parse_args()
    try:
        if args.hitung:
            tampilkan_campuran()
        elif args.port:
            if not 0.0 < args.gas <= 1.0:
                raise SystemExit("--gas harus lebih dari 0 dan paling besar 1.0")
            if args.detik <= 0:
                raise SystemExit("--detik harus lebih dari 0")
            jalankan_tes(args.port, args.baud, args.gas, args.detik)
        else:
            parser.print_help()
    except KeyboardInterrupt:
        print("\nDihentikan.")


if __name__ == "__main__":
    main()