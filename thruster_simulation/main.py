class Thruster:
    def __init__(self, name):
        self.name = name
        self.output = 0

    def set_output(self, value):
        # batasi value pada -100 sampai +100
        self.output = max(-100, min(100, value))

    def stop(self):
        self.output = 0

class ASVController:
    def __init__(self):
        self.thrusters = {
            "FL": Thruster("FL"),
            "FR": Thruster("FR"),
            "RL": Thruster("RL"),
            "RR": Thruster("RR"),
        }
        self.status = "STOP"

    def forward(self, speed):
        for t in self.thrusters.values():
            t.set_output(speed)
        self.status = "FORWARD"

    def backward(self, speed):
        for t in self.thrusters.values():
            t.set_output(-speed)
        self.status = "BACKWARD"

    def turn_left(self, turn_value=30):
        self.thrusters["FL"].set_output(-turn_value)
        self.thrusters["FR"].set_output(turn_value)
        self.thrusters["RL"].set_output(-turn_value)
        self.thrusters["RR"].set_output(turn_value)
        self.status = "TURN LEFT"

    def turn_right(self, turn_value=30):
        self.thrusters["FL"].set_output(turn_value)
        self.thrusters["FR"].set_output(-turn_value)
        self.thrusters["RL"].set_output(turn_value)
        self.thrusters["RR"].set_output(-turn_value)
        self.status = "TURN RIGHT"

    def stop(self):
        for t in self.thrusters.values():
            t.stop()
        self.status = "STOP"

    def emergency_stop(self):
        for t in self.thrusters.values():
            t.stop()
        self.status = "EMERGENCY STOP"
        print("!!! EMERGENCY STOP: semua thruster 0 !!!")

    def display_status(self):
        print(f"Status: {self.status}")
        for kode, t in self.thrusters.items():
            print(f"  {kode}: {t.output:+d}")

def read_speed():
    raw = input("Masukkan kecepatan (0-100): ").strip()
    try:
        speed = int(raw)
    except ValueError:
        print("Input tidak valid: masukkan angka bulat.")
        return None
    if speed < 0 or speed > 100:
        print("Kecepatan harus di antara 0 dan 100.")
        return None
    return speed


def print_menu():
    print("\n===== MENU KONTROL ASV =====")
    print("1. Forward")
    print("2. Backward")
    print("3. Turn Left")
    print("4. Turn Right")
    print("5. Stop")
    print("6. Emergency Stop")
    print("7. Exit")


def main():
    controller = ASVController()
    while True:
        print_menu()
        pilihan = input("Pilih menu (1-7): ").strip()

        if pilihan == "1":
            speed = read_speed()
            if speed is not None:
                controller.forward(speed)
                controller.display_status()
        elif pilihan == "2":
            speed = read_speed()
            if speed is not None:
                controller.backward(speed)
                controller.display_status()
        elif pilihan == "3":
            controller.turn_left()
            controller.display_status()
        elif pilihan == "4":
            controller.turn_right()
            controller.display_status()
        elif pilihan == "5":
            controller.stop()
            controller.display_status()
        elif pilihan == "6":
            controller.emergency_stop()
            controller.display_status()
        elif pilihan == "7":
            controller.stop()
            print("Program selesai.")
            break
        else:
            print("Pilihan tidak valid. Masukkan angka 1-7.")


if __name__ == "__main__":
    main()